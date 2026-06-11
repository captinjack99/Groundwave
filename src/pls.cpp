/**
 * @file pls.cpp
 * @brief Physical Layer Signaling encode/decode + ModCod detector
 *
 * PLS bit layout (32 bits per copy, transmitted twice):
 *
 *   Byte 0: [mod:3][fec:4][vcm_active:1]
 *   Byte 1: [vcm_slot:5][vcm_total_hi:3]
 *   Byte 2: [vcm_total_lo:2][reserved:6]
 *   Byte 3: [crc8:8]
 *
 * CRC-8 uses polynomial 0x07 (x^8 + x^2 + x + 1), init 0xFF,
 * computed over bytes 0-2 (24 bits).
 */

#include "pls.hpp"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <vector>

namespace gw {

// =========================================================================
// CRC-8
// =========================================================================

uint8_t plsCRC8(const uint8_t* data, size_t bytes) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < bytes; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80)
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            else
                crc = static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

// =========================================================================
// Shared payload packing + first-order Reed-Muller RM(1,4) FEC
// =========================================================================

namespace {

// Pack a PLSBlock into the 4-byte raw layout (byte 3 = CRC-8 over bytes 0-2).
// Single source of truth shared by the legacy and coded encoders.
void packRaw(const PLSBlock& pls, uint8_t raw[4]) {
    uint8_t mod_val = static_cast<uint8_t>(pls.modulation) & 0x07;
    uint8_t fec_val = static_cast<uint8_t>(pls.fec_rate) & 0x0F;
    uint8_t vcm_act = pls.vcm_active ? 1 : 0;
    uint8_t slot    = pls.vcm_slot & 0x1F;
    uint8_t total_in = (pls.vcm_total > 0 ? pls.vcm_total : 1);
    if (total_in > 32) total_in = 32;
    uint8_t total   = static_cast<uint8_t>((total_in - 1) & 0x1F);  // bias-1 (#31)

    raw[0] = static_cast<uint8_t>((mod_val << 5) | (fec_val << 1) | vcm_act);
    raw[1] = static_cast<uint8_t>((slot << 3) | ((total >> 2) & 0x07));
    raw[2] = static_cast<uint8_t>((total & 0x03) << 6);
    raw[3] = plsCRC8(raw, 3);
}

// Unpack the 4-byte raw layout into a PLSBlock, validating CRC + ranges.
bool unpackRaw(const uint8_t raw[4], PLSBlock& pls) {
    if (raw[3] != plsCRC8(raw, 3)) return false;

    uint8_t mod_val  = (raw[0] >> 5) & 0x07;
    uint8_t fec_val  = (raw[0] >> 1) & 0x0F;
    uint8_t vcm_act  = raw[0] & 0x01;
    uint8_t slot     = (raw[1] >> 3) & 0x1F;
    uint8_t total_hi = raw[1] & 0x07;
    uint8_t total_lo = (raw[2] >> 6) & 0x03;
    uint8_t total    = static_cast<uint8_t>(((total_hi << 2) | total_lo) + 1);

    if (mod_val > static_cast<uint8_t>(Modulation::QAM4096)) return false;
    if (fec_val > 10 && fec_val != 255) return false;

    pls.modulation = static_cast<Modulation>(mod_val);
    pls.fec_rate   = static_cast<FECRate>(fec_val);
    pls.vcm_active = (vcm_act != 0);
    pls.vcm_slot   = slot;
    pls.vcm_total  = total;
    pls.valid      = true;
    return true;
}

// --- First-order Reed-Muller RM(1,4) = [16, 5, 8] ---
// Message m = (m0..m4); codeword[j] = m0 ^ (m1&b0(j)) ^ ... ^ (m4&b3(j)),
// j ∈ [0,16). Minimum distance 8 → corrects 3 hard errors; the soft decoder
// is near-ML.
constexpr int RM_M = 4;
constexpr int RM_N = 16;   // 2^RM_M
constexpr int RM_K = 5;    // RM_M + 1

void rmEncodeBlock(const uint8_t msg[RM_K], uint8_t out[RM_N]) {
    for (int j = 0; j < RM_N; ++j) {
        int bit = msg[0];
        for (int i = 0; i < RM_M; ++i)
            bit ^= (msg[1 + i] & ((j >> i) & 1));
        out[j] = static_cast<uint8_t>(bit & 1);
    }
}

// Soft-decode 16 soft values (s[j] > 0 ⇒ bit 0) → 5 message bits, via the
// fast Walsh-Hadamard transform: H[a] = Σ_j (-1)^<a,j> s[j]. The correlation
// of s with the codeword of linear part a and offset m0 is (1-2·m0)·H[a], so
// the ML message is a* = argmax_a|H[a]|, m0 = (H[a*] < 0).
void rmDecodeBlock(const float s[RM_N], uint8_t msg_out[RM_K]) {
    float h[RM_N];
    for (int j = 0; j < RM_N; ++j) h[j] = s[j];
    for (int len = 1; len < RM_N; len <<= 1) {
        for (int i = 0; i < RM_N; i += (len << 1)) {
            for (int j = i; j < i + len; ++j) {
                float a = h[j], b = h[j + len];
                h[j] = a + b;
                h[j + len] = a - b;
            }
        }
    }
    int best = 0;
    float best_mag = -1.f;
    for (int a = 0; a < RM_N; ++a) {
        float m = std::abs(h[a]);
        if (m > best_mag) { best_mag = m; best = a; }
    }
    msg_out[0] = (h[best] >= 0.f) ? 0 : 1;
    for (int i = 0; i < RM_M; ++i)
        msg_out[1 + i] = static_cast<uint8_t>((best >> i) & 1);
}

} // anonymous

// =========================================================================
// PLS Encode
// =========================================================================

void encodePLS(const PLSBlock& pls, std::vector<uint8_t>& bits_out) {
    uint8_t raw[4] = {0, 0, 0, 0};
    packRaw(pls, raw);
    // Output: two copies of the 4 bytes = 8 bytes = 64 bits
    bits_out.resize(8);
    std::memcpy(bits_out.data(), raw, 4);
    std::memcpy(bits_out.data() + 4, raw, 4);
}

// =========================================================================
// PLS Decode
// =========================================================================

namespace {

bool parsePLSCopy(const uint8_t* raw, PLSBlock& pls) {
    return unpackRaw(raw, pls);
}

} // anonymous

bool decodePLS(const std::vector<uint8_t>& bits_in, PLSBlock& pls) {
    pls.valid = false;

    if (bits_in.size() < 8) return false;

    // Try first copy
    PLSBlock pls1, pls2;
    bool ok1 = parsePLSCopy(bits_in.data(), pls1);
    bool ok2 = parsePLSCopy(bits_in.data() + 4, pls2);

    if (ok1 && ok2) {
        // Both copies passed CRC-8 — require them to AGREE before trusting
        // the result (with only two copies we can't majority-vote, but we
        // can detect a CRC-8 collision / disagreement and reject). The two
        // serialized copies are byte-identical by construction, so any
        // mismatch means at least one copy is corrupt-but-CRC-valid. (#33)
        if (std::memcmp(bits_in.data(), bits_in.data() + 4, 4) != 0) {
            pls.valid = false;
            return false;
        }
        pls = pls1;
        return true;
    } else if (ok1) {
        pls = pls1;
        return true;
    } else if (ok2) {
        pls = pls2;
        return true;
    }

    return false;
}

// =========================================================================
// Coded PLS — Reed-Muller FEC + soft-combining
// =========================================================================

void encodePLSCoded(const PLSBlock& pls, std::vector<uint8_t>& coded_bits) {
    uint8_t raw[4] = {0, 0, 0, 0};
    packRaw(pls, raw);

    // 32 info bits (MSB-first), padded to PLS_RM_BLOCKS*RM_K (=35) with zeros.
    uint8_t info[PLS_RM_BLOCKS * RM_K] = {0};
    for (int i = 0; i < 32; ++i)
        info[i] = static_cast<uint8_t>((raw[i >> 3] >> (7 - (i & 7))) & 1);

    coded_bits.assign(PLS_CODED_BITS, 0);
    for (size_t b = 0; b < PLS_RM_BLOCKS; ++b) {
        uint8_t msg[RM_K];
        for (int k = 0; k < RM_K; ++k)
            msg[k] = info[b * RM_K + static_cast<size_t>(k)];
        uint8_t cw[RM_N];
        rmEncodeBlock(msg, cw);
        for (int j = 0; j < RM_N; ++j)
            coded_bits[b * RM_N + static_cast<size_t>(j)] = cw[j];
    }
}

bool decodePLSSoft(const std::vector<float>& soft, PLSBlock& pls) {
    pls.valid = false;
    if (soft.size() < PLS_CODED_BITS) return false;

    // Soft-combine the repeated copies that fit in the symbol (MRC: sum the
    // per-bit soft values — equal noise per copy, same OFDM symbol).
    const size_t n_copies = soft.size() / PLS_CODED_BITS;
    std::vector<float> comb(PLS_CODED_BITS, 0.f);
    for (size_t c = 0; c < n_copies; ++c)
        for (size_t i = 0; i < PLS_CODED_BITS; ++i)
            comb[i] += soft[c * PLS_CODED_BITS + i];

    // RM-decode the seven blocks → 35 info bits → first 32 → raw[4].
    uint8_t info[PLS_RM_BLOCKS * RM_K] = {0};
    for (size_t b = 0; b < PLS_RM_BLOCKS; ++b) {
        float s[RM_N];
        for (int j = 0; j < RM_N; ++j)
            s[j] = comb[b * RM_N + static_cast<size_t>(j)];
        uint8_t msg[RM_K];
        rmDecodeBlock(s, msg);
        for (int k = 0; k < RM_K; ++k)
            info[b * RM_K + static_cast<size_t>(k)] = msg[k];
    }

    uint8_t raw[4] = {0, 0, 0, 0};
    for (int i = 0; i < 32; ++i)
        if (info[i])
            raw[i >> 3] |= static_cast<uint8_t>(1 << (7 - (i & 7)));

    return unpackRaw(raw, pls);
}

// =========================================================================
// ModCodDetector
// =========================================================================

ModCodDetector::ModCodDetector(uint8_t confirm_count)
    : confirm_count_(confirm_count > 0 ? confirm_count : 1)
{
}

ModCodDetector::~ModCodDetector() = default;

bool ModCodDetector::feed(const PLSBlock& pls) {
    if (!pls.valid) return false;

    // Always update VCM slot info (this changes every frame in VCM mode)
    vcm_active_ = pls.vcm_active;
    vcm_slot_   = pls.vcm_slot;
    vcm_total_  = pls.vcm_total;

    // First valid PLS — initialize
    if (!initialized_) {
        current_mod_ = pls.modulation;
        current_fec_ = pls.fec_rate;
        pending_mod_ = pls.modulation;
        pending_fec_ = pls.fec_rate;
        match_count_ = confirm_count_;
        initialized_ = true;
        changed_     = true;
        return true;
    }

    // Check if this matches the current active modcod
    bool same_as_current = (pls.modulation == current_mod_ &&
                            pls.fec_rate == current_fec_);

    if (same_as_current) {
        // In VCM mode, the ModCod changes per-slot by design, so
        // we should NOT require confirmation for VCM slot changes.
        // But if VCM is not active and modcod matches, nothing to do.
        if (vcm_active_) {
            // VCM: modcod for this slot matches what we last set.
            // The "changed" flag should be set if the slot changed,
            // since the RX needs to reconfigure per-slot.
            return false;
        }
        // Reset pending tracking
        pending_mod_ = current_mod_;
        pending_fec_ = current_fec_;
        match_count_ = 0;
        return false;
    }

    // ModCod differs from current
    if (vcm_active_) {
        // In VCM mode, per-slot ModCod changes are expected and
        // immediate — no confirmation needed, the PLS is authoritative.
        current_mod_ = pls.modulation;
        current_fec_ = pls.fec_rate;
        changed_ = true;
        return true;
    }

    // Non-VCM: require consecutive confirmations before switching
    bool same_as_pending = (pls.modulation == pending_mod_ &&
                            pls.fec_rate == pending_fec_);

    if (same_as_pending) {
        ++match_count_;
        if (match_count_ >= confirm_count_) {
            current_mod_ = pending_mod_;
            current_fec_ = pending_fec_;
            changed_ = true;
            match_count_ = 0;
            return true;
        }
    } else {
        // New candidate — reset confirmation counter
        pending_mod_ = pls.modulation;
        pending_fec_ = pls.fec_rate;
        match_count_ = 1;
    }

    return false;
}

void ModCodDetector::reset() {
    current_mod_ = Modulation::QPSK;
    current_fec_ = FECRate::Rate_1_2;
    vcm_active_  = false;
    vcm_slot_    = 0;
    vcm_total_   = 1;
    changed_     = false;
    pending_mod_ = Modulation::QPSK;
    pending_fec_ = FECRate::Rate_1_2;
    match_count_ = 0;
    initialized_ = false;
}

} // namespace gw
