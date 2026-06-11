/**
 * @file hierarchical_mod.cpp
 * @brief Generic hierarchical modulation implementation
 *
 * Constellation construction (generic for any QAM order + HP/LP split):
 *
 * For any square/rectangular QAM with total_bps = hp_bits + lp_bits:
 *   - I-axis gets ceil(total_bps/2) bits, Q-axis gets floor(total_bps/2)
 *   - HP bits are split as evenly as possible between I and Q axes
 *   - LP bits fill the remainder on each axis
 *
 * Per-axis PAM construction:
 *   axis_value = α × PAM(hp_val, M_hp) × M_lp + PAM(lp_val, M_lp)
 *
 * Where PAM(index, M) returns Gray-coded PAM-M levels: -(M-1), ..., +(M-1).
 * At α=1 this reduces to standard uniform QAM. At α>1, HP bits have
 * increased minimum distance (robust layer) at the cost of LP bits.
 *
 * Gray coding is maintained per-axis per-layer for optimal soft demapping.
 */

#include "hierarchical_mod.hpp"
#include <cmath>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <cfloat>

namespace gw {

// =========================================================================
// Helpers
// =========================================================================

namespace {

// Extract bits from byte array (MSB first). Returns uint16_t: a single
// hierarchical layer can carry up to ~11 bits (e.g. QPSK/QAM4096 LP=10,
// QAM1024/QAM4096 HP=10, Custom up to 11), so a uint8_t result would
// silently drop the top bits and select the wrong constellation index.
inline uint16_t extractBits(const uint8_t* data, size_t bit_offset, uint8_t count) {
    uint16_t result = 0;
    for (uint8_t b = 0; b < count; ++b) {
        size_t pos = bit_offset + b;
        size_t byte_idx = pos / 8;
        uint8_t bit_idx = 7 - static_cast<uint8_t>(pos % 8);
        result = static_cast<uint16_t>((result << 1) |
                 ((data[byte_idx] >> bit_idx) & 1));
    }
    return result;
}

} // anonymous

// =========================================================================
// Gray decode
// =========================================================================

uint8_t HierarchicalMapper::grayDecode(uint8_t gray) {
    uint8_t bin = gray;
    for (uint8_t mask = static_cast<uint8_t>(gray >> 1); mask != 0;
         mask = static_cast<uint8_t>(mask >> 1)) {
        bin ^= mask;
    }
    return bin;
}

// =========================================================================
// Generic PAM level
// =========================================================================

float HierarchicalMapper::pamLevel(uint8_t index, uint8_t num_bits) {
    if (num_bits == 0) return 0.f;
    uint8_t M = static_cast<uint8_t>(1u << num_bits);
    // Gray decode the index to get natural ordering position
    uint8_t natural = grayDecode(index);
    // PAM-M level: 2*natural - (M-1), giving -(M-1),...,-1,+1,...,+(M-1)
    return static_cast<float>(2 * static_cast<int>(natural) -
                               static_cast<int>(M - 1));
}

// =========================================================================
// Construction
// =========================================================================

HierarchicalMapper::HierarchicalMapper(const HierarchicalConfig& cfg)
    : cfg_(cfg)
{
    if (cfg_.alpha < 1.0f) cfg_.alpha = 1.0f;

    // Resolve legacy presets to explicit hp/lp
    if (cfg_.mode != HierarchicalMode::None &&
        cfg_.mode != HierarchicalMode::Custom) {
        cfg_.hp_bits = cfg_.effectiveHP();
        cfg_.lp_bits = cfg_.effectiveLP();
    }

    if (isEnabled()) {
        buildConstellation();
    }
}

HierarchicalMapper::~HierarchicalMapper() = default;

// =========================================================================
// Generic constellation construction
// =========================================================================

void HierarchicalMapper::buildConstellation() {
    uint8_t hp_bps = hpBPS();
    uint8_t lp_bps = lpBPS();
    uint8_t total  = hp_bps + lp_bps;
    if (hp_bps == 0 || lp_bps == 0 || total < 2) return;

    // Split bits across I and Q axes
    uint8_t i_bits = static_cast<uint8_t>((total + 1) / 2); // ceil
    uint8_t q_bits = static_cast<uint8_t>(total / 2);        // floor

    // Distribute HP bits as evenly as possible between I and Q
    hp_i_ = static_cast<uint8_t>(std::min<uint8_t>((hp_bps + 1) / 2, i_bits));
    hp_q_ = static_cast<uint8_t>(hp_bps - hp_i_);

    // Ensure hp_q doesn't exceed q_bits; shift excess back to I
    if (hp_q_ > q_bits) {
        hp_i_ = static_cast<uint8_t>(hp_i_ + (hp_q_ - q_bits));
        hp_q_ = q_bits;
    }

    // LP bits fill the remainder
    lp_i_ = static_cast<uint8_t>(i_bits - hp_i_);
    lp_q_ = static_cast<uint8_t>(q_bits - hp_q_);

    // Constellation table: constellation_[hp_val][lp_val]
    size_t hp_count = size_t(1) << hp_bps;
    size_t lp_count = size_t(1) << lp_bps;

    constellation_.resize(hp_count);
    for (auto& v : constellation_) v.resize(lp_count);

    float alpha = cfg_.alpha;
    float M_lp_i = static_cast<float>(1u << lp_i_);  // LP PAM order on I
    float M_lp_q = static_cast<float>(1u << lp_q_);  // LP PAM order on Q

    // Generate all constellation points
    float total_power = 0.f;
    size_t total_points = 0;

    for (size_t hp = 0; hp < hp_count; ++hp) {
        // Decompose HP value into I-axis and Q-axis HP components
        // HP bits layout: [hp_i_ MSBs][hp_q_ LSBs]
        uint8_t hp_i_val = (hp_i_ > 0)
            ? static_cast<uint8_t>((hp >> hp_q_) & ((1u << hp_i_) - 1))
            : 0;
        uint8_t hp_q_val = (hp_q_ > 0)
            ? static_cast<uint8_t>(hp & ((1u << hp_q_) - 1))
            : 0;

        for (size_t lp = 0; lp < lp_count; ++lp) {
            // Decompose LP value into I-axis and Q-axis LP components
            // LP bits layout: [lp_i_ MSBs][lp_q_ LSBs]
            uint8_t lp_i_val = (lp_i_ > 0)
                ? static_cast<uint8_t>((lp >> lp_q_) & ((1u << lp_i_) - 1))
                : 0;
            uint8_t lp_q_val = (lp_q_ > 0)
                ? static_cast<uint8_t>(lp & ((1u << lp_q_) - 1))
                : 0;

            // I-axis: α × PAM(hp_i_val, M_hp_i) × M_lp_i + PAM(lp_i_val, M_lp_i)
            float i_val = alpha * pamLevel(hp_i_val, hp_i_) * M_lp_i
                        + pamLevel(lp_i_val, lp_i_);

            // Q-axis: α × PAM(hp_q_val, M_hp_q) × M_lp_q + PAM(lp_q_val, M_lp_q)
            float q_val = alpha * pamLevel(hp_q_val, hp_q_) * M_lp_q
                        + pamLevel(lp_q_val, lp_q_);

            ComplexSample s(i_val, q_val);
            constellation_[hp][lp] = s;
            total_power += std::norm(s);
            ++total_points;
        }
    }

    // Normalize for unit average power
    float avg_power = total_power / static_cast<float>(total_points);
    norm_factor_ = (avg_power > 1e-10f) ? 1.0f / std::sqrt(avg_power) : 1.0f;

    for (auto& hpv : constellation_) {
        for (auto& s : hpv) {
            s *= norm_factor_;
        }
    }
}

// =========================================================================
// Mapping
// =========================================================================

void HierarchicalMapper::map(const uint8_t* hp_bits, size_t hp_num_bits,
                              const uint8_t* lp_bits, size_t lp_num_bits,
                              ComplexBuf& out) const {
    if (!isEnabled()) return;

    uint8_t hp_bps = hpBPS();
    uint8_t lp_bps = lpBPS();
    if (hp_bps == 0 || lp_bps == 0) return;

    size_t hp_syms = hp_num_bits / hp_bps;
    size_t lp_syms = lp_num_bits / lp_bps;
    size_t num_syms = std::min(hp_syms, lp_syms);

    out.resize(num_syms);

    for (size_t i = 0; i < num_syms; ++i) {
        uint16_t hp_val = extractBits(hp_bits, i * hp_bps, hp_bps);
        uint16_t lp_val = extractBits(lp_bits, i * lp_bps, lp_bps);

        size_t hp_count = size_t(1) << hp_bps;
        size_t lp_count = size_t(1) << lp_bps;
        if (hp_val < hp_count && lp_val < lp_count) {
            out[i] = constellation_[hp_val][lp_val];
        } else {
            out[i] = ComplexSample(0.f, 0.f);
        }
    }
}

// =========================================================================
// Hard demapping
// =========================================================================

void HierarchicalMapper::demapHP(const ComplexBuf& symbols,
                                  std::vector<uint8_t>& hp_bits) const {
    if (!isEnabled()) return;

    uint8_t hp_bps = hpBPS();
    size_t total_bits = symbols.size() * hp_bps;
    size_t total_bytes = (total_bits + 7) / 8;
    hp_bits.assign(total_bytes, 0);

    for (size_t i = 0; i < symbols.size(); ++i) {
        // For HP, find the closest quadrant/region centroid
        size_t hp_count = size_t(1) << hp_bps;
        size_t lp_count = size_t(1) << lpBPS();

        uint16_t best_hp = 0;
        float best_dist = FLT_MAX;

        for (size_t hp = 0; hp < hp_count; ++hp) {
            // Compute centroid of all LP points for this HP value
            float ci = 0.f, cq = 0.f;
            for (size_t lp = 0; lp < lp_count; ++lp) {
                ci += constellation_[hp][lp].real();
                cq += constellation_[hp][lp].imag();
            }
            ci /= static_cast<float>(lp_count);
            cq /= static_cast<float>(lp_count);

            float di = symbols[i].real() - ci;
            float dq = symbols[i].imag() - cq;
            float dist = di * di + dq * dq;
            if (dist < best_dist) {
                best_dist = dist;
                best_hp = static_cast<uint16_t>(hp);
            }
        }

        // Pack bits
        for (uint8_t b = 0; b < hp_bps; ++b) {
            size_t bit_pos = i * hp_bps + b;
            size_t byte_idx = bit_pos / 8;
            uint8_t bit_idx = 7 - static_cast<uint8_t>(bit_pos % 8);
            uint8_t bit_val = (best_hp >> (hp_bps - 1 - b)) & 1;
            hp_bits[byte_idx] |= static_cast<uint8_t>(bit_val << bit_idx);
        }
    }
}

void HierarchicalMapper::demapLP(const ComplexBuf& symbols,
                                  std::vector<uint8_t>& lp_bits) const {
    if (!isEnabled()) return;

    uint8_t lp_bps = lpBPS();
    size_t total_bits = symbols.size() * lp_bps;
    size_t total_bytes = (total_bits + 7) / 8;
    lp_bits.assign(total_bytes, 0);

    size_t hp_count = size_t(1) << hpBPS();
    size_t lp_count = size_t(1) << lp_bps;

    for (size_t i = 0; i < symbols.size(); ++i) {
        // Find the closest constellation point (full ML search)
        uint16_t best_lp = 0;
        float best_dist = FLT_MAX;

        for (size_t hp = 0; hp < hp_count; ++hp) {
            for (size_t lp = 0; lp < lp_count; ++lp) {
                float di = symbols[i].real() - constellation_[hp][lp].real();
                float dq = symbols[i].imag() - constellation_[hp][lp].imag();
                float dist = di * di + dq * dq;
                if (dist < best_dist) {
                    best_dist = dist;
                    best_lp = static_cast<uint16_t>(lp);
                }
            }
        }

        // Pack bits
        for (uint8_t b = 0; b < lp_bps; ++b) {
            size_t bit_pos = i * lp_bps + b;
            size_t byte_idx = bit_pos / 8;
            uint8_t bit_idx = 7 - static_cast<uint8_t>(bit_pos % 8);
            uint8_t bit_val = (best_lp >> (lp_bps - 1 - b)) & 1;
            lp_bits[byte_idx] |= static_cast<uint8_t>(bit_val << bit_idx);
        }
    }
}

// =========================================================================
// Soft demapping
// =========================================================================

void HierarchicalMapper::demapSoftHP(const ComplexBuf& symbols,
                                      float noise_variance,
                                      std::vector<float>& llrs) const {
    if (!isEnabled()) return;

    uint8_t hp_bps = hpBPS();
    uint8_t lp_bps = lpBPS();
    size_t hp_count = size_t(1) << hp_bps;
    size_t lp_count = size_t(1) << lp_bps;

    llrs.resize(symbols.size() * hp_bps);

    float inv_var = (noise_variance > 1e-10f) ? 1.0f / noise_variance : 1e10f;

    for (size_t i = 0; i < symbols.size(); ++i) {
        ComplexSample r = symbols[i];

        for (uint8_t b = 0; b < hp_bps; ++b) {
            float min_dist_0 = FLT_MAX;
            float min_dist_1 = FLT_MAX;

            for (size_t hp = 0; hp < hp_count; ++hp) {
                uint8_t bit_val = (hp >> (hp_bps - 1 - b)) & 1;
                for (size_t lp = 0; lp < lp_count; ++lp) {
                    float di = r.real() - constellation_[hp][lp].real();
                    float dq = r.imag() - constellation_[hp][lp].imag();
                    float dist = di * di + dq * dq;
                    if (bit_val == 0 && dist < min_dist_0) min_dist_0 = dist;
                    if (bit_val == 1 && dist < min_dist_1) min_dist_1 = dist;
                }
            }

            // Convention: positive LLR ⇒ bit=0 more likely (matches LDPC + symbol_mapper).
            // LLR = log P(b=0|r) / P(b=1|r) ≈ (min_d1² - min_d0²) / σ²
            // Clamp to ±20 to match SymbolMapper's cap — mixing capped and
            // uncapped LLR scales into the same LDPC decoder skews the BP
            // (one layer's confident bits would dominate). (#42)
            float llr_hp = (min_dist_1 - min_dist_0) * inv_var;
            llrs[i * hp_bps + b] = std::max(-20.f, std::min(20.f, llr_hp));
        }
    }
}

void HierarchicalMapper::demapSoftLP(const ComplexBuf& symbols,
                                      float noise_variance,
                                      std::vector<float>& llrs) const {
    if (!isEnabled()) return;

    uint8_t hp_bps = hpBPS();
    uint8_t lp_bps = lpBPS();
    size_t hp_count = size_t(1) << hp_bps;
    size_t lp_count = size_t(1) << lp_bps;

    llrs.resize(symbols.size() * lp_bps);

    float inv_var = (noise_variance > 1e-10f) ? 1.0f / noise_variance : 1e10f;

    for (size_t i = 0; i < symbols.size(); ++i) {
        ComplexSample r = symbols[i];

        for (uint8_t b = 0; b < lp_bps; ++b) {
            float min_dist_0 = FLT_MAX;
            float min_dist_1 = FLT_MAX;

            for (size_t hp = 0; hp < hp_count; ++hp) {
                for (size_t lp = 0; lp < lp_count; ++lp) {
                    uint8_t bit_val = (lp >> (lp_bps - 1 - b)) & 1;
                    float di = r.real() - constellation_[hp][lp].real();
                    float dq = r.imag() - constellation_[hp][lp].imag();
                    float dist = di * di + dq * dq;
                    if (bit_val == 0 && dist < min_dist_0) min_dist_0 = dist;
                    if (bit_val == 1 && dist < min_dist_1) min_dist_1 = dist;
                }
            }

            // Convention: positive LLR ⇒ bit=0 more likely. Clamp to ±20
            // to match SymbolMapper (see demapSoftHP note). (#42)
            float llr_lp = (min_dist_1 - min_dist_0) * inv_var;
            llrs[i * lp_bps + b] = std::max(-20.f, std::min(20.f, llr_lp));
        }
    }
}

} // namespace gw
