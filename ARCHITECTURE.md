# Groundwave v2 — Ground-Up Rebuild

## Critical Bugs Found in v1

### 1. Preamble / Channel Estimation Fundamentally Broken
The preamble is generated as **time-domain** pseudo-random complex samples:
```cpp
// TX: generates time-domain samples directly
preamble_long_[i] = complex(cos(phase), sin(phase));
```
But `processPreamble()` takes the **FFT of received samples** and divides by
this time-domain reference as if it were a frequency-domain reference:
```cpp
fft_->forward(fft_input_, fft_output_);
h1[i] = fft_output_[i] / preamble_long_[i];  // WRONG: mixing domains
```
**Fix:** Preamble must be defined in the **frequency domain** (known BPSK
symbols on all active subcarriers), then IFFT'd for transmission. The receiver
FFTs the received preamble and divides by the known frequency-domain reference.

### 2. Transmitter Uses Legacy Real-Only Path
`buildAndModulateFrame()` calls `modulate()` and `generatePreamble()` which
return `SampleBuffer` (real float), **discarding the Q channel entirely**.
The complex baseband architecture was added to the modulator but never
connected to the transmitter or soundcard modem.

### 3. Frame Format TX/RX Mismatch
- TX `FrameBuilder::addPacket()` writes `[stream_id(1B)][length(1B)]`
- RX `FrameParser::parse()` reads `[packet_size(2B)]` (big-endian uint16)
These don't match. Different byte offsets cascade into total corruption.

### 4. Namespace Inconsistency
- `OFDMModulator` in `gw::`
- `OFDMDemodulator` in `gw::modulation::`
This causes confusing include paths and link errors.

### 5. GRAND `__builtin_popcount` — Not Portable
MSVC doesn't support `__builtin_popcount`. Need `_mm_popcnt_u32` or a
portable fallback.

### 6. Half-Implemented Subsystems Polluting Core
FBMC-OQAM, MIMO, polyphase filters, integer CFO, adaptive ModCod,
hierarchical modulation — all partially implemented, none integrated
into a working signal chain. These create compilation issues and
confuse the architecture.

---

## v2 Architecture Principles

1. **Complex baseband throughout** — No legacy real paths. Every DSP
   block operates on `std::complex<float>`.

2. **Frequency-domain preamble** — Known BPSK symbols defined in freq
   domain, IFFT'd for TX. RX FFTs received preamble, divides by known
   freq symbols. Correct by construction.

3. **One namespace** — Everything in `gw::`. No nested namespaces for
   core components.

4. **Incremental build** — Core chain first, loopback verified, then
   features added one at a time.

5. **MSVC-portable** — No GCC builtins. Use `<bit>` header (C++20) or
   portable fallbacks.

6. **Single frame format** — Unambiguous byte layout, documented in one
   place, used identically by TX and RX.

---

## Signal Chain

```
TX: Audio → Opus → FrameBuilder → LDPC Encode → QAM Map
    → OFDM Mod (IFFT+CP) → I/Q Upconvert → Soundcard

RX: Soundcard → I/Q Downconvert → Preamble Detect → CP Remove → FFT
    → Channel Estimate (pilots) → Equalize → QAM Soft Demap
    → LDPC Decode → FrameParser → Opus → Audio
```

## Frame Structure (bytes)

```
[Sync Word: 8B] [Header: 12B] [Payload: variable] [CRC32: 4B]

Header (12 bytes):
  [0]    version(4b) | num_packets(4b)
  [1-4]  frame_number (uint32 big-endian)
  [5]    fec_rate(4b) | modulation(4b)
  [6]    flags (reserved)
  [7-11] reserved

Payload packets:
  [stream_id: 1B] [length: 2B big-endian] [data: length bytes]
```

## Build Phases

### Phase 1: Core DSP Chain (this session)
- types.hpp — Core types, enums, constants
- fft_engine.hpp/cpp — Kiss FFT wrapper
- symbol_mapper.hpp/cpp — QAM map/demap with soft output
- ofdm.hpp/cpp — Modulator + Demodulator + Synchronizer
- frame.hpp/cpp — Frame builder + parser
- codec.hpp/cpp — Simple repetition/no-FEC placeholder
- loopback_test.cpp — End-to-end verification
- CMakeLists.txt

### Phase 2: FEC + Audio
- LDPC encoder/decoder (DVB-S2 matrices)
- Opus codec integration
- Bit interleaver

### Phase 3: Soundcard + I/Q
- Soundcard modem (miniaudio)
- I/Q up/downconversion
- AGC

### Phase 4: GUI
- Qt6 Widgets application (single implementation)
- Spectrum/constellation display
- Preset system

### Phase 5: Advanced Features
- Hierarchical modulation
- Adaptive ModCod
- PAPR reduction
