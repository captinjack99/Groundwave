/**
 * @file rs_chain_test.cpp
 * @brief Reed-Solomon outer-code integrity over the full chain + VCM per-slot
 *        FEC coverage. (#24 / #25 part — the RS-capacity + VCM-rebuild guard.)
 *
 * The live engine (gui/audio_engine.cpp) wraps the LDPC info block in a
 * Reed-Solomon(+16 parity) outer code so the rare residual byte errors that
 * survive the LDPC waterfall are mopped up before the frame is handed to the
 * CRC. The capacity arithmetic that decides whether RS is active
 *   (k_bytes <= 255 && k_bytes > PARITY_BYTES + 1, data = k_bytes - 16)
 * is mirrored in three places in audio_engine.cpp (TX wrap, RX unwrap, and
 * rebuildFEC capacity math). This test exercises that exact arithmetic at the
 * DSP level so a future regression there is caught without driving the Qt
 * AudioEngine.
 *
 * Tests:
 *   1. Full chain WITH RS:  info -> RS encode -> LDPC encode -> OFDM mod ->
 *      demod -> LDPC decode -> RS decode -> bit-exact on a clean channel.
 *   2. RS capacity:  injecting up to 8 byte-errors POST-LDPC is corrected by
 *      RS; 9+ errors are flagged uncorrectable (return -1), never silently
 *      "corrected" into garbage.
 *   3. VCM per-slot:  a small superframe (QPSK 1/2 x2 + QAM64 3/4 x2). Each
 *      slot's codeword, built with THAT slot's modcod, decodes with zero bit
 *      errors — catches per-slot FEC rebuild / capacity mismatches.
 *
 * Run with:  ./rs_chain_test
 * Exits non-zero if any assertion fails.
 */

#include "types.hpp"
#include "ofdm.hpp"
#include "ldpc.hpp"
#include "frame.hpp"
#include "interleaver.hpp"
#include "reed_solomon.hpp"
#include "pls.hpp"
#include "snr_calculator.hpp"  // modulationName / fecRateName

#include <cstdio>
#include <cstring>
#include <random>
#include <vector>
#include <algorithm>

using namespace dsca;

namespace {

int g_passed = 0;
int g_failed = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (cond) { ++g_passed; std::printf("  [PASS] %s\n", msg); }            \
        else      { ++g_failed; std::printf("  [FAIL] %s\n", msg); }            \
    } while (0)

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

// Mirror of audio_engine.cpp's RS-active predicate.
bool rsActive(size_t k_bytes) {
    return k_bytes <= ReedSolomon::MAX_BLOCK &&
           k_bytes > ReedSolomon::PARITY_BYTES + 1;
}

// =========================================================================
// Full TX->RX cycle WITH the Reed-Solomon outer code wrapped around the LDPC
// info block, exactly as audio_engine.cpp does it. Returns post-RS info-bit
// errors over the RS data region (-1 if a stage failed to build).
// =========================================================================
struct RSCycleResult {
    bool built       = false;
    bool rs_was_used = false;
    int  bit_errors  = -1;   // over the RS data region
    int  rs_corrected = 0;   // bytes RS reported corrected on RX
};

RSCycleResult runRSCycle(Modulation mod, FECRate fec, uint16_t fft,
                         uint32_t sample_rate, uint32_t seed,
                         int inject_byte_errors /* post-LDPC, 0 = none */) {
    RSCycleResult r{};

    OFDMParams ofdm;
    ofdm.fft_size      = fft;
    ofdm.modulation    = mod;
    ofdm.sample_rate   = sample_rate;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder ldpc_enc(fec, LDPCBlockSize::Short);
    LDPCDecoder ldpc_dec(fec, LDPCBlockSize::Short, 50);
    BitInterleaver inter(ldpc_enc.codewordBits());
    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);
    ReedSolomon rs;

    const size_t k_bytes = ldpc_enc.infoBytes();
    const size_t n_bytes = ldpc_enc.codewordBytes();

    // ---- RS capacity arithmetic (mirrors audio_engine.cpp) ----
    const bool rs_on = rsActive(k_bytes);
    const size_t rs_data_len = rs_on ? (k_bytes - ReedSolomon::PARITY_BYTES)
                                     : k_bytes;
    r.rs_was_used = rs_on;

    // ---- Build a frame that fits inside the RS data region ----
    size_t cap = (rs_data_len > constants::FRAME_OVERHEAD)
                 ? rs_data_len - constants::FRAME_OVERHEAD : 0;
    if (cap < 8) return r;

    std::mt19937 rng(seed);
    FrameBuilder fb(cap);
    std::vector<uint8_t> packet_data(cap - 4, 0);
    for (auto& b : packet_data) b = static_cast<uint8_t>(rng() & 0xFF);
    fb.addPacket(0, packet_data.data(), packet_data.size());
    auto frame_bytes = fb.build(/*frame_no*/ 7, fec, mod);

    // ---- TX: pad to RS data region, RS-encode in place, pad to k_bytes ----
    frame_bytes.resize(rs_data_len, 0);
    // Keep the clean reference of the RS data region for the final compare.
    std::vector<uint8_t> ref_data(frame_bytes.begin(), frame_bytes.end());
    if (rs_on) {
        frame_bytes.resize(k_bytes, 0);
        rs.encode(frame_bytes.data(), rs_data_len);
    }
    frame_bytes.resize(k_bytes, 0);
    // The clean transmitted RS codeword (data | parity) — the byte-exact block
    // a perfect LDPC decode recovers. Used as the injection base so the RS
    // capacity test isn't perturbed by LDPC partial-final-byte overspill (when
    // k is not a multiple of 8, the decoder's last info byte carries undefined
    // low bits that the channel-recovered `decoded` buffer can't match).
    std::vector<uint8_t> tx_rs_block(frame_bytes.begin(), frame_bytes.end());

    // ---- LDPC encode + interleave + OFDM modulate ----
    std::vector<uint8_t> codeword(n_bytes, 0);
    if (!ldpc_enc.encode(frame_bytes.data(), codeword.data())) return r;
    std::vector<uint8_t> ileaved(n_bytes, 0);
    inter.interleave(codeword.data(), ileaved.data());

    ComplexBuf tx_bb;
    tx_mod.modulateBits(ileaved.data(), ldpc_enc.codewordBits(), tx_bb);
    ComplexBuf preamble = tx_mod.generatePreamble();

    // ---- "Channel": clean baseband (preamble + payload) ----
    ComplexBuf channel = preamble;
    channel.insert(channel.end(), tx_bb.begin(), tx_bb.end());

    // ---- RX: preamble, demod, deinterleave, LDPC decode ----
    size_t sym_len     = ofdm.symbolLength();
    size_t short_total = 10 * (ofdm.fft_size / 4);
    if (channel.size() < short_total + 2 * sym_len) return r;
    ComplexBuf long_syms(channel.begin() + static_cast<ptrdiff_t>(short_total),
                         channel.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return r;

    size_t data_off = preamble.size();
    size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return r;
    size_t syms_per_cw = (ldpc_enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;

    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        size_t off = data_off + s * sym_len;
        if (off + sym_len > channel.size()) break;
        ComplexBuf one(channel.begin() + static_cast<ptrdiff_t>(off),
                       channel.begin() + static_cast<ptrdiff_t>(off + sym_len));
        std::vector<float> llrs;
        rx_demod.demodulateSoft(one, llrs, rx_demod.noiseVariance());
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(ldpc_enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    std::vector<uint8_t> decoded(k_bytes, 0);
    ldpc_dec.decode(deint.data(), decoded.data());

    // ---- Inject post-LDPC byte errors (simulating residual byte errors that
    //      survive the LDPC waterfall) into the RS codeword region ----
    // When injecting, start from the clean transmitted RS block (a perfect
    // LDPC recovery) so EXACTLY `inject_byte_errors` byte errors are present —
    // this is the residual-byte-error regime RS exists to mop up, and it
    // isolates the RS-capacity property from LDPC last-byte overspill. When
    // NOT injecting (inject_byte_errors == 0) we use the genuine
    // channel-recovered `decoded` buffer, so Test 1 still validates the real
    // end-to-end LDPC -> RS round-trip.
    if (inject_byte_errors > 0 && rs_on) {
        std::memcpy(decoded.data(), tx_rs_block.data(), k_bytes);
        size_t total = k_bytes;  // data + parity
        std::vector<size_t> positions;
        std::uniform_int_distribution<size_t> pd(0, total - 1);
        while (static_cast<int>(positions.size()) < inject_byte_errors) {
            size_t p = pd(rng);
            if (std::find(positions.begin(), positions.end(), p) == positions.end())
                positions.push_back(p);
        }
        for (size_t p : positions) {
            uint8_t v;
            do { v = static_cast<uint8_t>(rng() & 0xFF); } while (v == 0);
            decoded[p] ^= v;
        }
    }

    // ---- RX RS unwrap (mirrors audio_engine.cpp) ----
    std::vector<uint8_t> data_out;
    if (rs_on) {
        std::vector<uint8_t> rs_block(k_bytes, 0);
        std::memcpy(rs_block.data(), decoded.data(),
                    std::min(decoded.size(), k_bytes));
        int n_corrected = rs.decode(rs_block.data(), rs_data_len);
        r.rs_corrected = n_corrected;
        if (n_corrected >= 0) {
            data_out.assign(rs_block.begin(),
                            rs_block.begin() + static_cast<ptrdiff_t>(rs_data_len));
        } else {
            // Uncorrectable — pass raw LDPC output through (as the engine does).
            data_out.assign(decoded.begin(),
                            decoded.begin() + static_cast<ptrdiff_t>(rs_data_len));
        }
    } else {
        data_out.assign(decoded.begin(),
                        decoded.begin() + static_cast<ptrdiff_t>(rs_data_len));
    }

    // ---- Compare the recovered RS data region against the clean reference ----
    int errs = 0;
    size_t cmp_bits = rs_data_len * 8;
    for (size_t i = 0; i < cmp_bits; ++i)
        if (getBit(data_out.data(), i) != getBit(ref_data.data(), i)) ++errs;
    r.bit_errors = errs;
    r.built = true;
    return r;
}

// =========================================================================
// Test 1: full RS-wrapped chain round-trips bit-exact on a clean channel.
// =========================================================================
void test_rs_chain_clean() {
    std::printf("\n=== Test 1: RS+LDPC+OFDM clean-channel round-trip ===\n");
    struct MC { Modulation m; FECRate f; };
    MC cases[] = {
        { Modulation::QPSK,  FECRate::Rate_1_2 },
        { Modulation::QAM16, FECRate::Rate_3_4 },
        { Modulation::QAM64, FECRate::Rate_2_3 },
        { Modulation::QAM64, FECRate::Rate_3_4 },
    };
    for (auto& c : cases) {
        auto r = runRSCycle(c.m, c.f, 256, 48000, 0xC0DE1234u, /*inject*/ 0);
        char label[160];
        std::snprintf(label, sizeof(label),
                      "%-7s / %-5s : built=%d rs_used=%d err=%d "
                      "(RS data region bit-exact)",
                      modulationName(c.m), fecRateName(c.f),
                      r.built, r.rs_was_used, r.bit_errors);
        CHECK(r.built && r.rs_was_used && r.bit_errors == 0, label);
    }
}

// =========================================================================
// Test 2: RS capacity — <=8 post-LDPC byte errors corrected, 9+ flagged.
//
// This is the property audio_engine.cpp's rebuildFEC capacity math guards:
// RS(+16 parity) corrects up to t = PARITY/2 = 8 byte errors. We inject the
// errors POST-LDPC (after the inner decode) so RS is the only thing that can
// fix them — exactly the residual-byte-error regime RS exists for.
// =========================================================================
void test_rs_capacity() {
    std::printf("\n=== Test 2: RS capacity (<=8 corrected, 9+ uncorrectable) ===\n");
    const Modulation m = Modulation::QAM16;
    const FECRate    f = FECRate::Rate_3_4;

    // Up to 8 byte errors must be corrected → bit-exact recovery.
    for (int n_err = 1; n_err <= 8; ++n_err) {
        // Average over a few seeds so the gate doesn't hinge on one error layout.
        bool all_clean = true;
        int last_corrected = 0;
        for (uint32_t s = 0; s < 4; ++s) {
            auto r = runRSCycle(m, f, 256, 48000, 0x5A5A0000u + s * 0x9E37u + n_err,
                                /*inject*/ n_err);
            if (!r.built || r.bit_errors != 0) all_clean = false;
            last_corrected = r.rs_corrected;
        }
        char label[160];
        std::snprintf(label, sizeof(label),
                      "%d post-LDPC byte errors corrected by RS (last n_corrected=%d)",
                      n_err, last_corrected);
        CHECK(all_clean, label);
    }

    // 9+ byte errors must be flagged uncorrectable (n_corrected < 0), never a
    // silent spurious "correction" into a different valid-looking codeword.
    // (Inherent RS behavior CAN land on a wrong valid codeword for >t errors,
    //  but the post-correction syndrome recheck in reed_solomon.cpp (#40) makes
    //  the decoder REPORT failure; we assert that report fires.)
    for (int n_err : {9, 10, 12}) {
        int flagged = 0, trials = 0;
        for (uint32_t s = 0; s < 16; ++s) {
            auto r = runRSCycle(m, f, 256, 48000, 0xBEEF0000u + s * 0x9E37u + n_err,
                                /*inject*/ n_err);
            if (!r.built) continue;
            ++trials;
            if (r.rs_corrected < 0) ++flagged;
        }
        char label[160];
        std::snprintf(label, sizeof(label),
                      "%d errors flagged uncorrectable in %d/%d trials "
                      "(no silent corruption)",
                      n_err, flagged, trials);
        // Every over-capacity block must be flagged (n_corrected < 0). With
        // 9..12 errors against t=8 the syndrome recheck never produces a
        // false success.
        CHECK(trials > 0 && flagged == trials, label);
    }
}

// =========================================================================
// VCM per-slot codeword build/decode using THAT slot's modcod. This is the
// DSP-level equivalent of what the engine does each frame: read the slot's
// ModCod from the VCM schedule (via the PLS the detector recovers), rebuild
// the FEC for that modcod, and decode. We assert every slot's codeword
// decodes with zero bit errors on a clean channel — a per-slot FEC-rebuild or
// capacity mismatch would show up here as a non-zero error count for that slot.
// =========================================================================
struct SlotResult { bool built; int bit_errors; bool detector_locked; };

SlotResult runSlot(const VCMSlot& slot, uint16_t fft, uint32_t sr, uint32_t seed) {
    SlotResult r{false, -1, false};

    OFDMParams ofdm;
    ofdm.fft_size      = fft;
    ofdm.modulation    = slot.modulation;     // per-slot modcod
    ofdm.sample_rate   = sr;
    ofdm.cyclic_prefix = CyclicPrefix::CP_1_8;

    LDPCEncoder enc(slot.fec_rate, LDPCBlockSize::Short);
    LDPCDecoder dec(slot.fec_rate, LDPCBlockSize::Short, 50);
    BitInterleaver inter(enc.codewordBits());
    OFDMModulator   tx_mod(ofdm);
    OFDMDemodulator rx_demod(ofdm);

    std::mt19937 rng(seed);
    std::vector<uint8_t> info(enc.infoBytes(), 0);
    for (auto& b : info) b = static_cast<uint8_t>(rng() & 0xFF);
    std::vector<uint8_t> cw(enc.codewordBytes(), 0);
    if (!enc.encode(info.data(), cw.data())) return r;
    std::vector<uint8_t> ileaved(cw.size(), 0);
    inter.interleave(cw.data(), ileaved.data());

    ComplexBuf preamble = tx_mod.generatePreamble();
    ComplexBuf payload;
    tx_mod.modulateBits(ileaved.data(), enc.codewordBits(), payload);

    size_t short_total = 10 * (fft / 4);
    size_t sym_len = ofdm.symbolLength();
    if (preamble.size() < short_total + 2 * sym_len) return r;
    ComplexBuf long_syms(preamble.begin() + static_cast<ptrdiff_t>(short_total),
                         preamble.begin() + static_cast<ptrdiff_t>(short_total) + 2 * sym_len);
    if (!rx_demod.processPreamble(long_syms)) return r;

    size_t bits_per_ofdm = tx_mod.bitsPerOFDMSymbol();
    if (bits_per_ofdm == 0) return r;
    size_t syms_per_cw = (enc.codewordBits() + bits_per_ofdm - 1) / bits_per_ofdm;
    std::vector<float> all_llrs;
    for (size_t s = 0; s < syms_per_cw; ++s) {
        size_t off = s * sym_len;
        if (off + sym_len > payload.size()) break;
        ComplexBuf one(payload.begin() + static_cast<ptrdiff_t>(off),
                       payload.begin() + static_cast<ptrdiff_t>(off + sym_len));
        std::vector<float> llrs;
        rx_demod.demodulateSoft(one, llrs, rx_demod.noiseVariance());
        all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
    }
    all_llrs.resize(enc.codewordBits(), 0.f);

    std::vector<float> deint(all_llrs.size());
    inter.deinterleave(all_llrs.data(), deint.data());

    std::vector<uint8_t> out(enc.infoBytes(), 0);
    dec.decode(deint.data(), out.data());

    int errs = 0;
    for (size_t i = 0; i < enc.infoBits(); ++i)
        if (getBit(info.data(), i) != getBit(out.data(), i)) ++errs;
    r.bit_errors = errs;
    r.built = true;
    return r;
}

// =========================================================================
// Test 3: VCM per-slot FEC coverage. Schedule = QPSK/1-2 x2 + QAM64/3-4 x2.
// For each frame in the superframe: the schedule hands us a slot, the PLS for
// that frame round-trips through the coded-PLS codec and the ModCodDetector,
// and a full codeword built with that slot's modcod decodes with 0 errors.
// =========================================================================
void test_vcm_per_slot() {
    std::printf("\n=== Test 3: VCM per-slot codeword decode ===\n");
    VCMSchedule sched = createStereoVCM(
        Modulation::QPSK,  FECRate::Rate_1_2,   // robust PLP0 x2
        Modulation::QAM64, FECRate::Rate_3_4,   // enhance PLP1 x2
        /*n_robust*/ 2, /*n_enhance*/ 2);

    // The detector confirms after 1 matching PLS so a single frame flips the slot.
    ModCodDetector detector(/*confirm_count*/ 1);

    bool all_ok = true;
    for (uint32_t frame = 0; frame < sched.num_slots; ++frame) {
        const VCMSlot& slot = sched.slotForFrame(frame);

        // ---- PLS round-trip for this frame's slot (coded RM path) ----
        PLSBlock pls_tx = sched.plsForFrame(frame);
        std::vector<uint8_t> coded;
        encodePLSCoded(pls_tx, coded);
        // Map to BPSK soft values (bit 0 -> +large, bit 1 -> -large) and feed
        // the soft decoder directly on a clean "channel".
        std::vector<float> soft(coded.size());
        for (size_t i = 0; i < coded.size(); ++i)
            soft[i] = coded[i] ? -8.f : 8.f;
        PLSBlock pls_rx;
        bool pls_ok = decodePLSSoft(soft, pls_rx);
        detector.feed(pls_rx);

        bool slot_match = pls_ok &&
                          pls_rx.modulation == slot.modulation &&
                          pls_rx.fec_rate   == slot.fec_rate;

        // ---- Build + decode a codeword with this slot's modcod ----
        auto sr = runSlot(slot, 256, 48000, 0x5107u + frame * 0x9E37u);

        char label[200];
        std::snprintf(label, sizeof(label),
                      "slot %u (%-7s / %-5s, plp%u): pls_ok=%d slot_match=%d "
                      "codeword err=%d",
                      frame, modulationName(slot.modulation),
                      fecRateName(slot.fec_rate), slot.plp_id,
                      pls_ok ? 1 : 0, slot_match ? 1 : 0, sr.bit_errors);
        bool ok = slot_match && sr.built && sr.bit_errors == 0;
        CHECK(ok, label);
        all_ok = all_ok && ok;
    }

    // The detector must end locked onto the LAST slot's modcod (it tracked the
    // VCM hops correctly), and report VCM active.
    char dl[160];
    std::snprintf(dl, sizeof(dl),
                  "detector tracked superframe: final %-7s / %-5s vcm_active=%d "
                  "slot=%u/%u",
                  modulationName(detector.currentModulation()),
                  fecRateName(detector.currentFECRate()),
                  detector.vcmActive() ? 1 : 0,
                  detector.vcmSlot(), detector.vcmTotal());
    const VCMSlot& last = sched.slotForFrame(sched.num_slots - 1);
    CHECK(detector.currentModulation() == last.modulation &&
          detector.currentFECRate()    == last.fec_rate &&
          detector.vcmActive(), dl);
    (void)all_ok;
}

} // anonymous

int main() {
    std::printf("=== DSCA-NG RS + VCM Chain Test Suite ===\n");
    test_rs_chain_clean();
    test_rs_capacity();
    test_vcm_per_slot();
    std::printf("\n=== Result: %d passed, %d failed ===\n", g_passed, g_failed);
    return g_failed == 0 ? 0 : 1;
}
