/**
 * @file snr_calculator.hpp
 * @brief SNR thresholds, FM SCA channel model, and range/link budget calculator
 *
 * Provides three tiers of analysis for broadcast engineers:
 *
 * 1. **ModCod SNR Thresholds**: minimum Es/N0 (dB) required for quasi-error-free
 *    (QEF, BER < 1e-7) operation at each modulation+FEC combination.
 *    Based on Shannon limit + measured LDPC implementation loss.
 *
 * 2. **FM SCA Channel Model**: translates RF-level SNR at the receiver
 *    into SCA baseband SNR, accounting for:
 *      - FM demodulation gain (Carson's rule bandwidth × deviation ratio)
 *      - SCA injection level relative to main carrier
 *      - SCA subcarrier demodulation loss
 *      - De-emphasis and filtering losses
 *
 * 3. **Link Budget / Range Calculator**: given transmitter ERP, antenna
 *    heights, frequency, and terrain type, computes:
 *      - Free-space or Hata-model path loss
 *      - Received signal level
 *      - RF SNR → SCA SNR → margin for each ModCod
 *      - Maximum range for each ModCod
 *
 * Reference: ITU-R P.1546, Okumura-Hata model, ETSI EN 302 755 (DVB-T2),
 *            FCC 73.319 (SCA technical standards)
 */
#pragma once

#include "types.hpp"
#include "app_state.hpp"
#include "hierarchical_mod.hpp"
#include <vector>
#include <string>
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>

namespace gw {

// =========================================================================
// Naming helpers
// =========================================================================

inline const char* modulationName(Modulation m) {
    switch (m) {
        case Modulation::BPSK:    return "BPSK";
        case Modulation::QPSK:    return "QPSK";
        case Modulation::QAM16:   return "16-QAM";
        case Modulation::QAM64:   return "64-QAM";
        case Modulation::QAM256:  return "256-QAM";
        case Modulation::QAM1024: return "1024-QAM";
        case Modulation::QAM4096: return "4096-QAM";
        default: return "???";
    }
}

inline const char* fecRateName(FECRate r) {
    switch (r) {
        case FECRate::Rate_1_4:  return "1/4";
        case FECRate::Rate_1_3:  return "1/3";
        case FECRate::Rate_2_5:  return "2/5";
        case FECRate::Rate_1_2:  return "1/2";
        case FECRate::Rate_3_5:  return "3/5";
        case FECRate::Rate_2_3:  return "2/3";
        case FECRate::Rate_3_4:  return "3/4";
        case FECRate::Rate_4_5:  return "4/5";
        case FECRate::Rate_5_6:  return "5/6";
        case FECRate::Rate_8_9:  return "8/9";
        case FECRate::Rate_9_10: return "9/10";
        default: return "???";
    }
}

// =========================================================================
// ModCod SNR Threshold
// =========================================================================

/**
 * SNR threshold for quasi-error-free operation at a given ModCod.
 *
 * The threshold is Es/N0 in dB — energy per symbol to noise spectral
 * density ratio. This is the minimum SNR at the OFDM demodulator input
 * for BER < 1e-7 after LDPC decoding.
 *
 * Computed as: Shannon_limit(spectral_eff) + implementation_loss
 *
 * Shannon limit for coded modulation:
 *   C = R × log2(M)  bits/sym  (spectral efficiency)
 *   Es/N0_min = 2^C - 1  (linear)
 *   Es/N0_min_dB = 10 × log10(2^C - 1)
 *
 * Implementation loss depends on code rate and decoder:
 *   - Short LDPC block: +1.5 to +3.0 dB above Shannon
 *   - ORBGRAND: ~0.3 dB less than BP at short blocks
 *   - Lower code rates have less implementation loss
 */
struct ModCodThreshold {
    Modulation modulation;
    FECRate    fec_rate;
    float      shannon_limit_db;    ///< Theoretical minimum Es/N0 (dB)
    float      impl_loss_db;        ///< LDPC implementation loss (dB)
    float      threshold_db;        ///< Required Es/N0 = shannon + impl_loss
    float      spectral_eff;        ///< Coded spectral efficiency (bits/sym)
};

/**
 * Compute the SNR threshold for any ModCod combination.
 */
inline ModCodThreshold computeThreshold(Modulation mod, FECRate fec) {
    ModCodThreshold t;
    t.modulation = mod;
    t.fec_rate   = fec;

    float bps = static_cast<float>(bitsPerSymbol(mod));
    float R   = codeRateValue(fec);
    t.spectral_eff = bps * R;

    // Shannon limit: Es/N0_min = 2^C - 1 (linear)
    float shannon_lin = std::pow(2.0f, t.spectral_eff) - 1.0f;
    t.shannon_limit_db = (shannon_lin > 1e-10f)
        ? 10.0f * std::log10(shannon_lin) : -10.0f;

    // Implementation loss model (empirical, based on DVB-S2 short FECFRAME):
    //   - Base loss: 1.8 dB for rate 1/2
    //   - Lower rates: less loss (better threshold convergence)
    //   - Higher rates: more loss (less redundancy, harder to decode)
    //   - Higher QAM: slight additional loss from demapper approximation
    float rate_factor = 1.0f + 1.5f * (R - 0.5f); // 0.625 at R=1/4, 1.75 at R=9/10
    if (rate_factor < 0.5f) rate_factor = 0.5f;

    float qam_factor = 0.0f;
    if (bps > 4) qam_factor = 0.1f * (bps - 4.0f); // +0.2 for QAM64, +0.8 for QAM4096

    t.impl_loss_db = 1.2f + rate_factor + qam_factor;
    t.threshold_db = t.shannon_limit_db + t.impl_loss_db;

    return t;
}

/**
 * Compute thresholds for ALL valid ModCod combinations.
 * Returns a sorted vector (ascending by threshold_db).
 */
inline std::vector<ModCodThreshold> computeAllThresholds() {
    std::vector<ModCodThreshold> all;

    static const Modulation mods[] = {
        Modulation::BPSK, Modulation::QPSK, Modulation::QAM16,
        Modulation::QAM64, Modulation::QAM256, Modulation::QAM1024,
        Modulation::QAM4096
    };
    static const FECRate rates[] = {
        FECRate::Rate_1_4, FECRate::Rate_1_3, FECRate::Rate_2_5,
        FECRate::Rate_1_2, FECRate::Rate_3_5, FECRate::Rate_2_3,
        FECRate::Rate_3_4, FECRate::Rate_4_5, FECRate::Rate_5_6,
        FECRate::Rate_8_9, FECRate::Rate_9_10
    };

    for (auto m : mods) {
        for (auto r : rates) {
            all.push_back(computeThreshold(m, r));
        }
    }

    // Sort by threshold
    std::sort(all.begin(), all.end(),
              [](const ModCodThreshold& a, const ModCodThreshold& b) {
                  return a.threshold_db < b.threshold_db;
              });
    return all;
}

/**
 * Compute SNR thresholds for hierarchical HP and LP layers.
 *
 * The HP layer threshold is based on the effective HP constellation
 * (reduced-order QAM), while LP requires the full constellation SNR.
 * The α parameter creates an SNR gap between layers.
 *
 * HP effective SNR gain: 20·log10(α) dB above uniform QAM threshold
 * LP effective SNR penalty: depends on α and constellation geometry
 */
struct HierThreshold {
    float hp_threshold_db;    ///< Minimum SNR for HP layer QEF
    float lp_threshold_db;    ///< Minimum SNR for LP layer QEF
    float hp_spectral_eff;    ///< HP coded spectral efficiency
    float lp_spectral_eff;    ///< LP coded spectral efficiency
    float coverage_gain_db;   ///< HP coverage improvement over uniform (dB)
};

inline HierThreshold computeHierThreshold(
        const HierarchicalConfig& hier,
        FECRate hp_fec, FECRate lp_fec) {
    HierThreshold h{};

    uint8_t hp_bps = hier.effectiveHP();
    uint8_t lp_bps = hier.effectiveLP();
    uint8_t total  = hp_bps + lp_bps;
    if (hp_bps == 0 || lp_bps == 0) return h;

    // Guard against a non-physical alpha (<= 0, or NaN) from a hand-edited or
    // loaded config: 20*log10(alpha) and the LP-penalty log10 would otherwise
    // be -inf / NaN and poison the displayed HP/LP thresholds. alpha is the
    // HP/LP distance ratio (>= 1 physically; 1 = uniform). The dialog clamps to
    // [1,4], but config load does not, so be self-defending here.
    float alpha = (hier.alpha > 1e-2f) ? hier.alpha : 1e-2f;

    // HP layer: effectively decoding a reduced-order constellation
    // The HP quadrant spacing is α times larger than LP spacing
    // HP threshold ≈ threshold for (2^hp_bps)-QAM at the HP FEC rate
    //                minus the α gain: 20·log10(α) dB
    float hp_R = codeRateValue(hp_fec);
    h.hp_spectral_eff = static_cast<float>(hp_bps) * hp_R;
    float hp_shan = std::pow(2.0f, h.hp_spectral_eff) - 1.0f;
    float hp_shan_db = (hp_shan > 1e-10f) ? 10.0f * std::log10(hp_shan) : -10.0f;
    float alpha_gain_db = 20.0f * std::log10(alpha);
    h.hp_threshold_db = hp_shan_db + 2.0f - alpha_gain_db;

    // LP layer: needs full constellation SNR + penalty from non-uniform spacing
    // LP sees the combined constellation — threshold is near the uniform
    // total-QAM threshold at the LP FEC rate, plus a penalty for reduced
    // LP minimum distance: ~20·log10((α·M_lp + 1)/(M_lp + 1))
    float lp_R = codeRateValue(lp_fec);
    h.lp_spectral_eff = static_cast<float>(lp_bps) * lp_R;
    float full_eff = static_cast<float>(total) * lp_R;
    float full_shan = std::pow(2.0f, full_eff) - 1.0f;
    float full_shan_db = (full_shan > 1e-10f) ? 10.0f * std::log10(full_shan) : -10.0f;
    float M_lp = static_cast<float>(1u << (lp_bps / 2));
    float lp_penalty = 20.0f * std::log10((alpha * M_lp + 1.0f) / (M_lp + 1.0f));
    h.lp_threshold_db = full_shan_db + 2.5f + lp_penalty;

    // Uniform reference: what threshold would total-QAM at hp_fec need?
    float uniform_eff = static_cast<float>(total) * hp_R;
    float uniform_shan = std::pow(2.0f, uniform_eff) - 1.0f;
    float uniform_db = (uniform_shan > 1e-10f) ? 10.0f * std::log10(uniform_shan) : -10.0f;
    h.coverage_gain_db = (uniform_db + 2.0f) - h.hp_threshold_db;

    return h;
}

// =========================================================================
// FM SCA Channel Model
// =========================================================================

/** FM SCA subcarrier parameters */
struct SCAChannelParams {
    float carrier_freq_mhz  = 98.0f;   ///< FM carrier frequency (MHz)
    float sca_freq_khz      = 67.0f;   ///< SCA subcarrier frequency (kHz): 67 or 92
    float sca_deviation_khz = 7.5f;    ///< SCA frequency deviation (kHz)
    float sca_injection_pct = 10.0f;   ///< SCA injection as % of total deviation
    float main_deviation_khz = 75.0f;  ///< Main channel peak deviation (kHz)
    float if_bandwidth_khz  = 200.0f;  ///< FM receiver IF bandwidth (kHz)
    float deemph_loss_db    = 2.0f;    ///< De-emphasis and filtering loss (dB)
    float receiver_nf_db    = 6.0f;    ///< Receiver noise figure (dB)
    float receiver_bw_khz   = 200.0f;  ///< Receiver noise bandwidth (kHz)
    float antenna_gain_dbi  = 0.0f;    ///< Receive antenna gain (dBi)
};

/**
 * Compute the SCA baseband SNR from the RF signal-to-noise ratio.
 *
 * FM demodulation provides SNR improvement (FM gain) above threshold:
 *   SNR_fm = 3 × (BW_IF / BW_audio)² × (Δf / BW_audio)²
 *
 * The SCA subcarrier gets a fraction of the total deviation, so:
 *   SNR_sca = SNR_fm × (sca_injection / 100)² × (BW_audio / BW_sca)
 *             - deemphasis_loss
 *
 * @param rf_snr_db    RF SNR at the receiver input (dB)
 * @param sca          SCA channel parameters
 * @return SCA baseband SNR in dB
 */
inline float rfToScaSNR(float rf_snr_db, const SCAChannelParams& sca) {
    // FM demodulation gain (above FM threshold)
    // FM threshold is approximately 10-12 dB RF SNR
    float fm_threshold_db = 10.0f;

    if (rf_snr_db < fm_threshold_db) {
        // Below FM threshold — FM capture effect collapses
        // SCA SNR degrades rapidly (roughly 3:1 in dB)
        return rf_snr_db - 30.0f; // Essentially unusable
    }

    // FM demod gain: 3 × (BW_IF/BW_sca) × (Δf_sca/BW_sca)²
    float bw_sca = 2.0f * sca.sca_deviation_khz; // Carson's rule for SCA
    float dev_ratio = sca.sca_deviation_khz / bw_sca;
    float bw_ratio = sca.if_bandwidth_khz / bw_sca;
    float fm_gain_lin = 3.0f * bw_ratio * dev_ratio * dev_ratio;
    float fm_gain_db = 10.0f * std::log10(fm_gain_lin);

    // SCA injection loss: 20·log10(injection_pct/100)
    float injection_loss_db = 20.0f * std::log10(sca.sca_injection_pct / 100.0f);

    // SCA baseband SNR
    float sca_snr_db = rf_snr_db + fm_gain_db + injection_loss_db
                       - sca.deemph_loss_db;

    return sca_snr_db;
}

/**
 * Compute RF SNR required to achieve a target SCA baseband SNR.
 * Inverse of rfToScaSNR().
 */
inline float scaToRfSNR(float sca_snr_db, const SCAChannelParams& sca) {
    float bw_sca = 2.0f * sca.sca_deviation_khz;
    float dev_ratio = sca.sca_deviation_khz / bw_sca;
    float bw_ratio = sca.if_bandwidth_khz / bw_sca;
    float fm_gain_lin = 3.0f * bw_ratio * dev_ratio * dev_ratio;
    float fm_gain_db = 10.0f * std::log10(fm_gain_lin);
    float injection_loss_db = 20.0f * std::log10(sca.sca_injection_pct / 100.0f);

    return sca_snr_db - fm_gain_db - injection_loss_db + sca.deemph_loss_db;
}

// =========================================================================
// Propagation Models
// =========================================================================

/** Terrain/environment type for Hata model */
enum class TerrainType : uint8_t {
    FreeSpace = 0,    ///< Free-space path loss (theoretical best case)
    OpenRural = 1,    ///< Open/rural area
    Suburban  = 2,    ///< Suburban area
    Urban     = 3,    ///< Urban area (small/medium city)
    DenseUrban = 4,   ///< Dense urban (large city)
};

inline const char* terrainName(TerrainType t) {
    switch (t) {
        case TerrainType::FreeSpace:  return "Free Space";
        case TerrainType::OpenRural:  return "Open/Rural";
        case TerrainType::Suburban:   return "Suburban";
        case TerrainType::Urban:      return "Urban";
        case TerrainType::DenseUrban: return "Dense Urban";
        default: return "???";
    }
}

/** Propagation model parameters */
struct PropagationParams {
    float erp_watts      = 1000.0f;  ///< Effective radiated power (Watts)
    float freq_mhz       = 98.0f;    ///< Carrier frequency (MHz)
    float tx_height_m    = 100.0f;   ///< TX antenna height (meters AGL)
    float rx_height_m    = 2.0f;     ///< RX antenna height (meters AGL)
    float rx_antenna_dbi = 0.0f;     ///< RX antenna gain (dBi)
    float cable_loss_db  = 2.0f;     ///< Feedline/connector loss (dB)
    TerrainType terrain  = TerrainType::Suburban;
};

/**
 * Free-space path loss (FSPL) in dB.
 * @param distance_km  Distance in kilometers
 * @param freq_mhz     Frequency in MHz
 */
inline float freeSpacePathLoss(float distance_km, float freq_mhz) {
    if (distance_km <= 0.f || freq_mhz <= 0.f) return 0.f;
    // FSPL = 20·log10(d_km) + 20·log10(f_MHz) + 32.44
    return 20.0f * std::log10(distance_km)
         + 20.0f * std::log10(freq_mhz)
         + 32.44f;
}

/**
 * Okumura-Hata path loss model (150-1500 MHz, 1-20 km).
 * Extended with corrections for different terrain types.
 *
 * @param distance_km  Distance (km), valid range 1-20 km
 * @param freq_mhz     Frequency (MHz), valid range 150-1500
 * @param hb_m         Base station height (m), valid range 30-200
 * @param hm_m         Mobile height (m), valid range 1-10
 * @param terrain      Terrain type
 * @return Path loss in dB
 */
inline float hataPathLoss(float distance_km, float freq_mhz,
                           float hb_m, float hm_m, TerrainType terrain) {
    // Clamp to model validity range (extrapolate gracefully outside)
    float d = std::max(0.1f, distance_km);
    float f = std::max(100.0f, std::min(1500.0f, freq_mhz));
    float hb = std::max(30.0f, std::min(200.0f, hb_m));
    float hm = std::max(1.0f, std::min(10.0f, hm_m));

    // Small/medium city mobile antenna correction
    float a_hm = (1.1f * std::log10(f) - 0.7f) * hm
               - (1.56f * std::log10(f) - 0.8f);

    // Base Hata formula (urban, small/medium city)
    float L_urban = 69.55f + 26.16f * std::log10(f)
                  - 13.82f * std::log10(hb) - a_hm
                  + (44.9f - 6.55f * std::log10(hb)) * std::log10(d);

    switch (terrain) {
        case TerrainType::FreeSpace:
            return freeSpacePathLoss(d, f);

        case TerrainType::OpenRural:
            // Hata open area correction
            return L_urban - 4.78f * std::log10(f) * std::log10(f)
                   + 18.33f * std::log10(f) - 40.94f;

        case TerrainType::Suburban:
            // Hata suburban correction
            return L_urban - 2.0f * std::pow(std::log10(f / 28.0f), 2.0f) - 5.4f;

        case TerrainType::Urban:
            return L_urban;

        case TerrainType::DenseUrban: {
            // Large city correction for mobile antenna
            float a_hm_large;
            if (f >= 400.0f) {
                a_hm_large = 3.2f * std::pow(std::log10(11.75f * hm), 2.0f) - 4.97f;
            } else {
                a_hm_large = 8.29f * std::pow(std::log10(1.54f * hm), 2.0f) - 1.1f;
            }
            return L_urban + (a_hm - a_hm_large); // Replace correction
        }

        default:
            return L_urban;
    }
}

// =========================================================================
// Link Budget
// =========================================================================

/** Result of a link budget calculation for one ModCod */
struct LinkBudgetEntry {
    Modulation modulation;
    FECRate    fec_rate;
    float      threshold_db;     ///< Required SCA SNR (dB)
    float      margin_db;        ///< SNR margin at reference distance
    float      max_range_km;     ///< Maximum range for QEF (km)
    float      net_bitrate_bps;  ///< Net data rate (bps)
    float      spectral_eff;     ///< Spectral efficiency (bits/sym)
    bool       viable;           ///< True if achievable at reference distance
};

/** Complete link budget result */
struct LinkBudget {
    // Input parameters
    PropagationParams prop;
    SCAChannelParams  sca;
    float ref_distance_km = 10.0f;   ///< Reference distance for margin calc

    // Computed RF values
    float erp_dbw       = 0.f;       ///< ERP in dBW
    float erp_dbm       = 0.f;       ///< ERP in dBm
    float path_loss_db  = 0.f;       ///< Path loss at ref distance
    float rx_level_dbm  = 0.f;       ///< Received signal level at ref distance
    float noise_floor_dbm = 0.f;     ///< Receiver noise floor
    float rf_snr_db     = 0.f;       ///< RF SNR at ref distance
    float sca_snr_db    = 0.f;       ///< SCA baseband SNR at ref distance
    float fm_gain_db    = 0.f;       ///< FM demodulation gain

    // Per-ModCod results
    std::vector<LinkBudgetEntry> entries;

    // Hierarchical results (if applicable)
    bool has_hier = false;
    HierThreshold hier_thresh;
    float hier_hp_margin_db  = 0.f;
    float hier_lp_margin_db  = 0.f;
    float hier_hp_range_km   = 0.f;
    float hier_lp_range_km   = 0.f;
};

/**
 * Compute the full link budget.
 *
 * @param prop    Propagation parameters
 * @param sca     SCA channel parameters
 * @param ref_km  Reference distance for margin calculations
 * @param ofdm    OFDM parameters (for net bitrate computation)
 * @param hier    Optional hierarchical config (nullptr = skip)
 * @param hp_fec  HP FEC rate (for hierarchical)
 * @param lp_fec  LP FEC rate (for hierarchical)
 */
inline LinkBudget computeLinkBudget(
        const PropagationParams& prop,
        const SCAChannelParams& sca,
        float ref_km,
        const OFDMParams& ofdm,
        const HierarchicalConfig* hier = nullptr,
        FECRate hp_fec = FECRate::Rate_1_2,
        FECRate lp_fec = FECRate::Rate_3_4) {
    LinkBudget lb;
    lb.prop = prop;
    lb.sca  = sca;
    lb.ref_distance_km = ref_km;

    // ERP
    lb.erp_dbw = 10.0f * std::log10(std::max(0.001f, prop.erp_watts));
    lb.erp_dbm = lb.erp_dbw + 30.0f;

    // Path loss at reference distance
    if (prop.terrain == TerrainType::FreeSpace) {
        lb.path_loss_db = freeSpacePathLoss(ref_km, prop.freq_mhz);
    } else {
        lb.path_loss_db = hataPathLoss(ref_km, prop.freq_mhz,
                                        prop.tx_height_m, prop.rx_height_m,
                                        prop.terrain);
    }

    // Received signal level
    lb.rx_level_dbm = lb.erp_dbm - lb.path_loss_db
                    + prop.rx_antenna_dbi - prop.cable_loss_db;

    // Noise floor: kTB + NF
    // k = -174 dBm/Hz, B in Hz
    float noise_bw_hz = sca.receiver_bw_khz * 1000.0f;
    lb.noise_floor_dbm = -174.0f + 10.0f * std::log10(noise_bw_hz)
                       + sca.receiver_nf_db;

    // RF SNR
    lb.rf_snr_db = lb.rx_level_dbm - lb.noise_floor_dbm;

    // SCA baseband SNR
    lb.sca_snr_db = rfToScaSNR(lb.rf_snr_db, sca);

    // FM gain for display
    float bw_sca = 2.0f * sca.sca_deviation_khz;
    float dev_ratio = sca.sca_deviation_khz / bw_sca;
    float bw_ratio = sca.if_bandwidth_khz / bw_sca;
    lb.fm_gain_db = 10.0f * std::log10(3.0f * bw_ratio * dev_ratio * dev_ratio);

    // Compute OFDM-derived net bitrate parameters
    auto cp = ComputedParams::compute(ofdm, FrameParams{});
    float sym_rate = (cp.symbol_duration_ms > 0.f)
        ? 1000.0f / cp.symbol_duration_ms : 0.f;
    float data_syms = static_cast<float>(cp.data_subcarriers);

    // Per-ModCod entries
    auto all_thresh = computeAllThresholds();
    lb.entries.reserve(all_thresh.size());

    for (const auto& t : all_thresh) {
        LinkBudgetEntry e;
        e.modulation    = t.modulation;
        e.fec_rate      = t.fec_rate;
        e.threshold_db  = t.threshold_db;
        e.spectral_eff  = t.spectral_eff;
        e.margin_db     = lb.sca_snr_db - t.threshold_db;
        e.viable        = (e.margin_db >= 0.f);

        // Net bitrate: data_subcarriers × bps × code_rate × symbol_rate
        float bps = static_cast<float>(bitsPerSymbol(t.modulation));
        e.net_bitrate_bps = data_syms * bps * codeRateValue(t.fec_rate) * sym_rate;

        // Max range: find distance where SCA SNR = threshold
        // Binary search over distance
        float lo = 0.1f, hi = 500.0f;
        for (int iter = 0; iter < 40; ++iter) {
            float mid = (lo + hi) / 2.0f;
            float pl;
            if (prop.terrain == TerrainType::FreeSpace) {
                pl = freeSpacePathLoss(mid, prop.freq_mhz);
            } else {
                pl = hataPathLoss(mid, prop.freq_mhz,
                                   prop.tx_height_m, prop.rx_height_m,
                                   prop.terrain);
            }
            float rx = lb.erp_dbm - pl + prop.rx_antenna_dbi - prop.cable_loss_db;
            float rf_snr = rx - lb.noise_floor_dbm;
            float sca_snr = rfToScaSNR(rf_snr, sca);

            if (sca_snr > t.threshold_db)
                lo = mid;
            else
                hi = mid;
        }
        e.max_range_km = lo;

        lb.entries.push_back(e);
    }

    // Hierarchical thresholds
    if (hier && hier->enabled) {
        lb.has_hier = true;
        lb.hier_thresh = computeHierThreshold(*hier, hp_fec, lp_fec);
        lb.hier_hp_margin_db = lb.sca_snr_db - lb.hier_thresh.hp_threshold_db;
        lb.hier_lp_margin_db = lb.sca_snr_db - lb.hier_thresh.lp_threshold_db;

        // HP range (binary search)
        auto rangeSearch = [&](float thresh) -> float {
            float lo = 0.1f, hi = 500.0f;
            for (int iter = 0; iter < 40; ++iter) {
                float mid = (lo + hi) / 2.0f;
                float pl;
                if (prop.terrain == TerrainType::FreeSpace)
                    pl = freeSpacePathLoss(mid, prop.freq_mhz);
                else
                    pl = hataPathLoss(mid, prop.freq_mhz,
                                       prop.tx_height_m, prop.rx_height_m,
                                       prop.terrain);
                float rx = lb.erp_dbm - pl + prop.rx_antenna_dbi - prop.cable_loss_db;
                float sca_snr = rfToScaSNR(rx - lb.noise_floor_dbm, sca);
                if (sca_snr > thresh) lo = mid; else hi = mid;
            }
            return lo;
        };

        lb.hier_hp_range_km = rangeSearch(lb.hier_thresh.hp_threshold_db);
        lb.hier_lp_range_km = rangeSearch(lb.hier_thresh.lp_threshold_db);
    }

    return lb;
}

} // namespace gw
