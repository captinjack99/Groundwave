# Per-Stream M/S over Hierarchical Modulation — Architecture

## Design rationale

The previous implementation carried only stream 1's audio on the Side
(LP) codeword, duplicating it on the Mid (HP) codeword as well. This
was a half-measure: only one of the eight possible streams could benefit
from graceful stereo→mono degradation, and the Side copy was a mono mix
rather than the proper `(L − R)/2` stereo-difference signal.

The corrected design treats hierarchical modulation as a system-wide
unequal-error-protection (UEP) scheme: **every enabled stereo stream
splits into Mid = (L + R)/2 and Side = (L − R)/2 components; all Mid
components are coded onto the HP layer; all Side components are coded
onto the LP layer**. When LP fails, every stereo stream collapses to
mono simultaneously. Mono streams (channels=1) ride Mid only and are
not affected by LP failures at all.

This matches the canonical layered audio + UEP pattern from DVB-T2 /
broadcast literature (Hong & Vetterli 2020+, on rotated-constellation
UEP) and is the design that makes the "predictive Side reconstruction"
feature (Item 1 in `INVENTION_DISCLOSURE.md`) operate on real per-stream
audio rather than synthesized test tones.

## Signal flow (TX)

```
For each enabled stream i:
  If channels[i] == 2 (stereo):
      input PCM = L0 R0 L1 R1 … (interleaved)
      Mid_i  = (L + R) / 2  ──►  OpusEncoder mid_[i]   ──► opus_packet_mid_i
      Side_i = (L − R) / 2  ──►  OpusEncoder side_[i]  ──► opus_packet_side_i
  Else (channels[i] == 1, mono):
      input PCM = M0 M1 M2 …
      Mid_i  = input         ──►  OpusEncoder mid_[i]   ──► opus_packet_mid_i
      Side_i = (none — mono streams contribute no LP payload)

Mid Frame  := [SYNC | HDR | mid packets for all enabled streams  | CRC]
Side Frame := [SYNC | HDR | side packets for stereo streams only | CRC]

Mid Frame  ──► RS-16 ──► LDPC(rate r) ──► BitInterleaver M ──► HP bits
Side Frame ──► RS-16 ──► LDPC(rate r) ──► BitInterleaver S ──► LP bits

HierarchicalMapper(α): combines (HP, LP) bit groups → QAM constellation
                       point with α-controlled HP/LP energy split.

OFDM modulate → IQ upconvert → soundcard / RF chain.
```

When hierarchical mod is **off** (uniform constellation), only the Mid
frame is sent and stereo streams degrade to Mid-only (mono). This is
the correct default: users without hier mod still hear something
sensible on stereo streams.

## Signal flow (RX)

```
Soundcard / RF → IQ downconvert → OFDM demodulate → per-subcarrier
                                                     equalized symbols.

For each codeword's worth of symbols:
  HP LLRs := HierarchicalMapper::demapSoftHP(symbols, σ²)
  LP LLRs := HierarchicalMapper::demapSoftLP(symbols, σ²)

HP LLRs  ──► Deinterleave M ──► LDPC decode ──► RS decode ──► Mid Frame
LP LLRs  ──► Deinterleave S ──► LDPC decode ──► RS decode ──► Side Frame

Parse Mid Frame:  for each packet, decode Mid_i via OpusDecoder mid_[i]
Parse Side Frame: for each packet, decode Side_i via OpusDecoder side_[i]

For each stream i:
  If LP decoded AND LP-LLR confidence high:
      L_i  = Mid_i + Side_i_decoded
      R_i  = Mid_i − Side_i_decoded
  Else if LP decoded AND LP-LLR confidence marginal:
      Side_i := α · Side_i_decoded + (1 − α) · SideReconstructor(Mid_i)
      L_i = Mid_i + Side_i ;  R_i = Mid_i − Side_i
  Else (LP failed):
      Side_i := SideReconstructor(Mid_i)   ← predictive synthesis
      L_i = Mid_i + Side_i ;  R_i = Mid_i − Side_i

Write {L, R} to per-stream RX output rings.
Multi-channel audio_monitor routes ring i → output channel i.
```

## Bitrate budgeting

For a stream with total bitrate `T` and channels=2, the budget splits:

```
B_mid_i  = T_i × β_i        where β_i ∈ [0.55, 0.70] for music,
B_side_i = T_i × (1 − β_i)                  ≈ 0.50      for talk,
                                            ≈ 0.40      for mostly-mono
```

Default β = 0.62 (slightly Mid-heavy, matching perceptual stereo width
information density). Configurable per-stream via a new
`mid_side_split` field in `StreamConfig`.

For mono streams: `B_mid_i = T_i`, no Side.

The coordinator's existing `allocateBudget(total_bps)` proportionally
splits the total channel bandwidth among enabled streams by `weight`.
After that, each stream further splits its allocation into Mid and Side
encoders by `β`.

## Frame format

**Mid Frame** carries packets with `stream_id` 0–7 for all enabled
streams (channels=1 OR channels=2). The existing `FrameBuilder` and
`FrameParser` work unchanged.

**Side Frame** carries packets with `stream_id` 0–7 for stereo streams
only. Same format, different content. The two frames are independent
codewords carried on independent FEC chains, mapped to the HP/LP layers
of the same hierarchical OFDM symbol.

When hier mod is OFF, no Side Frame is built; the OFDM modulator emits
a uniform constellation carrying only the Mid codeword.

## Graceful degradation modes

The system has four discrete states:

| HP locked | LP locked | LP-LLR confidence | Output                  |
|-----------|-----------|-------------------|-------------------------|
| ✓         | ✓         | high (≥ 8 dB)     | Full stereo (M+S, M−S) |
| ✓         | ✓         | marginal (2–8 dB) | Crossfaded stereo       |
| ✓         | ✗         | n/a               | Synthesized stereo      |
| ✗         | —         | —                 | No output, sync alarm   |

The "synthesized stereo" mode uses the cascaded-all-pass decorrelator in
`SideReconstructor` to produce a plausible Side from the decoded Mid.
This is the key invention claim (see `INVENTION_DISCLOSURE.md` §5.A) —
its value is unlocked by the per-stream M/S architecture because every
stereo stream gets predictive recovery, not just stream 1.

## Implementation phases

1. **`MultiStreamCoordinator` API extension**: add `tx_side_[]` and
   `rx_side_[]` encoder/decoder arrays. New `encodeIntoFrames(mid_fb,
   side_fb)` method that produces packets for both frames. New
   `onParsedFrames(mid_pf, side_pf)` method that recombines L/R per
   stream and writes to stereo RX outputs.

2. **Engine TX path**: build two `FrameBuilder`s, two LDPC codewords,
   pass both to the hierarchical mapper. When hier OFF, drop the side
   frame.

3. **Engine RX path**: parse both ParsedFrames after the M/S branch's
   parallel LDPC decode. Pass both to coordinator's onParsedFrames.

4. **RX output**: per-stream rings become stereo (L + R) when the
   stream is configured channels=2. Audio_monitor's channel routing
   needs to know to interleave L,R per stream. Or — simpler — channel i
   carries L of stream i, channel i+8 carries R of stream i. (Limited
   by device channel count.)

5. **Bitrate allocator**: split each stream's allocation into Mid and
   Side bps according to β.

6. **GUI**: stream panel shows two bitrate readouts per stereo stream
   (Mid kbps, Side kbps). Status bar shows the global "stereo lock"
   state.

## Non-goals

- **Per-stream independent LP-LLR confidence**: the LP layer is a
  single LDPC codeword shared across all stereo streams' Side data.
  When LP fails, all stereo streams fall back together. Per-stream LLR
  weighting would require splitting the Side payload across multiple
  LDPC codewords, which is more complex than the perceptual benefit.

- **Different α per stream**: α is a constellation-level parameter,
  not a per-stream knob. The whole system runs at one α.

- **Adaptive Mid/Side split based on channel quality**: the user can
  configure β per stream offline, but no closed-loop adaptation. (In
  broadcast there's no feedback channel anyway.)

## State-of-the-art context

Synthesis from the 2020+ literature audit:

- **Audio**: Opus 1.5 (with optional DRED redundancy) remains the
  right codec choice for the 32–128 kbps stereo budget. Neural codecs
  (Lyra v2, EnCodec) win only at sub-12 kbps where we're not
  operating. xHE-AAC is patent-encumbered.

- **FEC**: Layered min-sum LDPC (current) is near-optimal for n=2160
  short blocks. The biggest remaining win is **BICM-ID** (iterative
  demap↔decode) — 1–2 dB at marginal SNR. Documented as future
  work; weeks of integration effort.

- **OFDM waveform**: Plain CP-OFDM with **windowed CP** is sufficient
  for soundcard passband. f-OFDM/UFMC/GFDM were proposed for 5G and
  largely abandoned. OTFS only matters if we ever go mobile/Doppler.

- **Channel estimation**: DFT-denoised MMSE + Wiener (current) is at
  the floor for AWGN+slow-phase channels. **Decision-directed
  refinement** would add ~0.3 dB — easy future addition.

- **Synchronization**: Zadoff-Chu (current) is still SOTA. Sub-0-dB
  acquisition is hard regardless of preamble design.

The per-stream M/S over hier-mod design with predictive Side recovery
is the genuinely novel contribution in this codebase (no published
prior art combines LDPC-posterior-LLR-driven crossfade between
transmitted and synthesized Side as a hierarchical-mod failure-recovery
mechanism).
