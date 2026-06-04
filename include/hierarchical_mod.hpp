/**
 * @file hierarchical_mod.hpp
 * @brief Hierarchical (layered) modulation for multi-priority streaming
 *
 * Implements DVB-T style hierarchical modulation where a single QAM
 * constellation encodes two separate bit streams:
 *
 *   - HP (High Priority): robust base layer (e.g., mono audio)
 *   - LP (Low Priority):  enhancement layer (e.g., stereo difference)
 *
 * The constellation is parameterized by α (alpha), which controls the
 * ratio of the minimum distance between HP quadrants to the minimum
 * distance within a quadrant:
 *
 *   α = d_HP / d_LP
 *
 * Supported modes:
 *   - QPSK/QAM16:  HP = 2 bits (QPSK quadrant), LP = 2 bits (position within)
 *   - QPSK/QAM64:  HP = 2 bits, LP = 4 bits
 *   - QAM16/QAM64: HP = 4 bits, LP = 2 bits
 *
 * The HP stream is decodable at lower SNR (even if LP is corrupted).
 * The LP stream requires higher SNR but carries additional capacity.
 *
 * For M/S stereo: HP carries Mid (mono-compatible), LP carries Side.
 *
 * Reference: ETSI EN 300 744 (DVB-T), Section 4.3.5
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <string>
#include <utility>

namespace dsca {

// =========================================================================
// Configuration
// =========================================================================

/** Hierarchical modulation mode */
enum class HierarchicalMode : uint8_t {
    None         = 0,    ///< Uniform (standard) modulation — no hierarchy
    QPSK_QAM16   = 1,    ///< HP: 2 bits (QPSK), LP: 2 bits → 16-QAM
    QPSK_QAM64   = 2,    ///< HP: 2 bits, LP: 4 bits → 64-QAM
    QAM16_QAM64  = 3,    ///< HP: 4 bits, LP: 2 bits → 64-QAM
    // QAM-256 base (8 bps total)
    QPSK_QAM256  = 4,    ///< HP: 2, LP: 6 → 256-QAM
    QAM16_QAM256 = 5,    ///< HP: 4, LP: 4 → 256-QAM (symmetric)
    QAM64_QAM256 = 6,    ///< HP: 6, LP: 2 → 256-QAM
    // QAM-1024 base (10 bps total)
    QPSK_QAM1024  = 7,   ///< HP: 2, LP: 8 → 1024-QAM
    QAM16_QAM1024 = 8,   ///< HP: 4, LP: 6 → 1024-QAM
    QAM64_QAM1024 = 9,   ///< HP: 6, LP: 4 → 1024-QAM
    QAM256_QAM1024 = 10, ///< HP: 8, LP: 2 → 1024-QAM
    // QAM-4096 base (12 bps total)
    QPSK_QAM4096   = 11, ///< HP: 2, LP: 10 → 4096-QAM
    QAM16_QAM4096  = 12, ///< HP: 4, LP: 8  → 4096-QAM
    QAM64_QAM4096  = 13, ///< HP: 6, LP: 6  → 4096-QAM (symmetric)
    QAM256_QAM4096 = 14, ///< HP: 8, LP: 4  → 4096-QAM
    QAM1024_QAM4096 = 15, ///< HP: 10, LP: 2 → 4096-QAM

    Custom      = 255,   ///< User-defined HP/LP split for any constellation
};

/** Resolve a preset mode into the equivalent uniform modulation of each
 *  hierarchical layer. Used by `hierarchicalModeName()` and the GUI to
 *  render unambiguous layer names ("HP=QPSK, LP=16-QAM") rather than
 *  the DVB-T "HP / total constellation" shorthand that users routinely
 *  mis-read. */
inline const char* hierLayerName(uint8_t bps) {
    switch (bps) {
        case 1:  return "BPSK";
        case 2:  return "QPSK";
        case 4:  return "16-QAM";
        case 6:  return "64-QAM";
        case 8:  return "256-QAM";
        case 10: return "1024-QAM";
        case 12: return "4096-QAM";
        default: return "—";
    }
}

inline const char* hierarchicalModeName(HierarchicalMode m) {
    // Names are now in "HP=X, LP=Y" form (the natural reading) instead
    // of the DVB-T "X-in-Y" shorthand. The constellation in the air is
    // X+Y bits per symbol; the user picks based on what HP and LP each
    // carry, not what the total looks like.
    switch (m) {
        case HierarchicalMode::None:            return "None (Uniform)";
        case HierarchicalMode::QPSK_QAM16:      return "HP=QPSK, LP=QPSK   (2+2 = 16-QAM)";
        case HierarchicalMode::QPSK_QAM64:      return "HP=QPSK, LP=16-QAM   (2+4 = 64-QAM)";
        case HierarchicalMode::QAM16_QAM64:     return "HP=16-QAM, LP=QPSK   (4+2 = 64-QAM)";
        case HierarchicalMode::QPSK_QAM256:     return "HP=QPSK, LP=64-QAM   (2+6 = 256-QAM)";
        case HierarchicalMode::QAM16_QAM256:    return "HP=16-QAM, LP=16-QAM   (4+4 = 256-QAM)";
        case HierarchicalMode::QAM64_QAM256:    return "HP=64-QAM, LP=QPSK   (6+2 = 256-QAM)";
        case HierarchicalMode::QPSK_QAM1024:    return "HP=QPSK, LP=256-QAM   (2+8 = 1024-QAM)";
        case HierarchicalMode::QAM16_QAM1024:   return "HP=16-QAM, LP=64-QAM   (4+6 = 1024-QAM)";
        case HierarchicalMode::QAM64_QAM1024:   return "HP=64-QAM, LP=16-QAM   (6+4 = 1024-QAM)";
        case HierarchicalMode::QAM256_QAM1024:  return "HP=256-QAM, LP=QPSK   (8+2 = 1024-QAM)";
        case HierarchicalMode::QPSK_QAM4096:    return "HP=QPSK, LP=1024-QAM   (2+10 = 4096-QAM)";
        case HierarchicalMode::QAM16_QAM4096:   return "HP=16-QAM, LP=256-QAM   (4+8 = 4096-QAM)";
        case HierarchicalMode::QAM64_QAM4096:   return "HP=64-QAM, LP=64-QAM   (6+6 = 4096-QAM)";
        case HierarchicalMode::QAM256_QAM4096:  return "HP=256-QAM, LP=16-QAM   (8+4 = 4096-QAM)";
        case HierarchicalMode::QAM1024_QAM4096: return "HP=1024-QAM, LP=QPSK   (10+2 = 4096-QAM)";
        case HierarchicalMode::Custom:          return "Custom";
        default: return "Unknown";
    }
}

/** Get HP bits per symbol for a hierarchical mode */
inline uint8_t hierHP_BPS(HierarchicalMode m, uint8_t custom_hp = 0) {
    switch (m) {
        case HierarchicalMode::QPSK_QAM16:      return 2;
        case HierarchicalMode::QPSK_QAM64:      return 2;
        case HierarchicalMode::QAM16_QAM64:     return 4;
        case HierarchicalMode::QPSK_QAM256:     return 2;
        case HierarchicalMode::QAM16_QAM256:    return 4;
        case HierarchicalMode::QAM64_QAM256:    return 6;
        case HierarchicalMode::QPSK_QAM1024:    return 2;
        case HierarchicalMode::QAM16_QAM1024:   return 4;
        case HierarchicalMode::QAM64_QAM1024:   return 6;
        case HierarchicalMode::QAM256_QAM1024:  return 8;
        case HierarchicalMode::QPSK_QAM4096:    return 2;
        case HierarchicalMode::QAM16_QAM4096:   return 4;
        case HierarchicalMode::QAM64_QAM4096:   return 6;
        case HierarchicalMode::QAM256_QAM4096:  return 8;
        case HierarchicalMode::QAM1024_QAM4096: return 10;
        case HierarchicalMode::Custom:          return custom_hp;
        default: return 0;
    }
}

/** Get LP bits per symbol for a hierarchical mode */
inline uint8_t hierLP_BPS(HierarchicalMode m, uint8_t custom_lp = 0) {
    switch (m) {
        case HierarchicalMode::QPSK_QAM16:      return 2;
        case HierarchicalMode::QPSK_QAM64:      return 4;
        case HierarchicalMode::QAM16_QAM64:     return 2;
        case HierarchicalMode::QPSK_QAM256:     return 6;
        case HierarchicalMode::QAM16_QAM256:    return 4;
        case HierarchicalMode::QAM64_QAM256:    return 2;
        case HierarchicalMode::QPSK_QAM1024:    return 8;
        case HierarchicalMode::QAM16_QAM1024:   return 6;
        case HierarchicalMode::QAM64_QAM1024:   return 4;
        case HierarchicalMode::QAM256_QAM1024:  return 2;
        case HierarchicalMode::QPSK_QAM4096:    return 10;
        case HierarchicalMode::QAM16_QAM4096:   return 8;
        case HierarchicalMode::QAM64_QAM4096:   return 6;
        case HierarchicalMode::QAM256_QAM4096:  return 4;
        case HierarchicalMode::QAM1024_QAM4096: return 2;
        case HierarchicalMode::Custom:          return custom_lp;
        default: return 0;
    }
}

/** Total bits per symbol in hierarchical mode */
inline uint8_t hierTotalBPS(HierarchicalMode m, uint8_t custom_hp = 0, uint8_t custom_lp = 0) {
    return hierHP_BPS(m, custom_hp) + hierLP_BPS(m, custom_lp);
}

struct HierarchicalConfig {
    HierarchicalMode mode    = HierarchicalMode::None;
    float            alpha   = 2.0f;  ///< HP/LP distance ratio (1 = uniform, 2 = DVB-T α=2, 4 = max protection)
    bool             enabled = false;

    // Custom mode: user-defined HP/LP bit split
    uint8_t          hp_bits = 0;     ///< HP bits per symbol (1..total-1)
    uint8_t          lp_bits = 0;     ///< LP bits per symbol (1..total-1)

    /** Effective HP bits (resolves legacy presets or custom) */
    uint8_t effectiveHP() const { return hierHP_BPS(mode, hp_bits); }
    /** Effective LP bits (resolves legacy presets or custom) */
    uint8_t effectiveLP() const { return hierLP_BPS(mode, lp_bits); }
    /** Total bits per symbol */
    uint8_t effectiveTotal() const { return effectiveHP() + effectiveLP(); }
};

// =========================================================================
// Validation helpers
// =========================================================================

/** Check if a hierarchical split is valid for a given modulation.
 *  Requirements: total_bps >= 2, hp >= 1, lp >= 1, hp+lp = total_bps. */
inline bool isValidHierSplit(Modulation mod, uint8_t hp, uint8_t lp) {
    uint8_t total = bitsPerSymbol(mod);
    if (total < 2) return false;          // BPSK can't do hierarchy
    if (hp < 1 || lp < 1) return false;   // Both layers need >= 1 bit
    if (hp + lp != total) return false;    // Must use all constellation bits
    return true;
}

/** Return all valid HP/LP splits for a modulation.
 *  Each pair is {hp_bits, lp_bits}. */
inline std::vector<std::pair<uint8_t, uint8_t>> validHierSplits(Modulation mod) {
    std::vector<std::pair<uint8_t, uint8_t>> splits;
    uint8_t total = bitsPerSymbol(mod);
    if (total < 2) return splits;
    for (uint8_t hp = 1; hp < total; ++hp) {
        splits.push_back({hp, static_cast<uint8_t>(total - hp)});
    }
    return splits;
}

/** Create a custom hierarchical config for any modulation + split */
inline HierarchicalConfig makeHierConfig(Modulation mod, uint8_t hp, float alpha = 2.0f) {
    HierarchicalConfig cfg;
    uint8_t total = bitsPerSymbol(mod);
    if (hp < 1 || hp >= total || total < 2) return cfg; // Invalid
    cfg.mode = HierarchicalMode::Custom;
    cfg.alpha = alpha;
    cfg.enabled = true;
    cfg.hp_bits = hp;
    cfg.lp_bits = total - hp;
    return cfg;
}

// =========================================================================
// HierarchicalMapper
// =========================================================================

class HierarchicalMapper {
public:
    /**
     * @param cfg  Hierarchical modulation configuration
     */
    explicit HierarchicalMapper(const HierarchicalConfig& cfg = HierarchicalConfig());
    ~HierarchicalMapper();

    /**
     * Map HP and LP bit streams to hierarchical constellation symbols.
     *
     * @param hp_bits      High-priority bit stream
     * @param hp_num_bits  Number of HP bits
     * @param lp_bits      Low-priority bit stream
     * @param lp_num_bits  Number of LP bits
     * @param out          Output constellation symbols
     *
     * The number of output symbols = min(hp_num_bits/hp_bps, lp_num_bits/lp_bps).
     * If streams are different lengths, the shorter one limits output.
     */
    void map(const uint8_t* hp_bits, size_t hp_num_bits,
             const uint8_t* lp_bits, size_t lp_num_bits,
             ComplexBuf& out) const;

    /**
     * Demap received symbols to HP bits (robust — ignores LP).
     * Only needs the quadrant information, so works at lower SNR.
     *
     * @param symbols   Received equalized symbols
     * @param hp_bits   Output HP hard-decision bits
     */
    void demapHP(const ComplexBuf& symbols, std::vector<uint8_t>& hp_bits) const;

    /**
     * Demap received symbols to LP bits (requires higher SNR).
     * Needs the full symbol position within each quadrant.
     *
     * @param symbols   Received equalized symbols
     * @param lp_bits   Output LP hard-decision bits
     */
    void demapLP(const ComplexBuf& symbols, std::vector<uint8_t>& lp_bits) const;

    /**
     * Soft demap HP stream — LLR output for FEC.
     *
     * @param symbols        Received equalized symbols
     * @param noise_variance Noise variance estimate (σ²)
     * @param llrs           Output LLRs (size = symbols.size() * hp_bps)
     */
    void demapSoftHP(const ComplexBuf& symbols, float noise_variance,
                     std::vector<float>& llrs) const;

    /**
     * Soft demap LP stream — LLR output for FEC.
     *
     * @param symbols        Received equalized symbols
     * @param noise_variance Noise variance estimate (σ²)
     * @param llrs           Output LLRs (size = symbols.size() * lp_bps)
     */
    void demapSoftLP(const ComplexBuf& symbols, float noise_variance,
                     std::vector<float>& llrs) const;

    /** Get the total bits per symbol */
    uint8_t totalBPS() const { return cfg_.effectiveTotal(); }
    uint8_t hpBPS()    const { return cfg_.effectiveHP(); }
    uint8_t lpBPS()    const { return cfg_.effectiveLP(); }

    const HierarchicalConfig& config() const { return cfg_; }
    bool isEnabled() const { return cfg_.enabled && cfg_.effectiveHP() > 0 && cfg_.effectiveLP() > 0; }

private:
    /** Build the constellation using generic PAM-based construction */
    void buildConstellation();

    /** Generic PAM level: Gray-coded, unit spacing */
    static float pamLevel(uint8_t index, uint8_t num_bits);

    /** Gray decode (binary reflected Gray → natural binary) */
    static uint8_t grayDecode(uint8_t gray);

    HierarchicalConfig cfg_;

    // I/Q axis bit allocation (derived from hp/lp split)
    uint8_t hp_i_ = 0;  ///< HP bits on I-axis
    uint8_t hp_q_ = 0;  ///< HP bits on Q-axis
    uint8_t lp_i_ = 0;  ///< LP bits on I-axis
    uint8_t lp_q_ = 0;  ///< LP bits on Q-axis

    // Precomputed constellation points: constellation_[hp_val][lp_val]
    std::vector<std::vector<ComplexSample>> constellation_;

    // Normalization factor (for unit average power)
    float norm_factor_ = 1.0f;
};

} // namespace dsca
