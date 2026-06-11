/**
 * @file pls.hpp
 * @brief Physical Layer Signaling (PLS) and Variable Coding & Modulation (VCM)
 *
 * === Physical Layer Signaling ===
 *
 * Solves the broadcast bootstrap problem: the receiver must know the
 * ModCod *before* it can demodulate/decode the data frame. The PLS block is
 * transmitted immediately after the preamble at a fixed, robust point: a
 * 32-bit payload protected by first-order Reed-Muller FEC, mapped to genuine
 * BPSK and repeated to fill the OFDM symbol's data subcarriers, then
 * soft-combined + FHT-decoded at the RX (see encodePLSCoded/decodePLSSoft).
 * This decodes well below the data tier's threshold.
 *
 * PLS block format (32-bit payload = 4 bytes; the engine sends it RM-coded):
 *
 *   Bits  Field
 *   ----  -----
 *   [0:2]   modulation (3 bits, maps to Modulation enum 0-6)
 *   [3:6]   fec_rate   (4 bits, maps to FECRate enum 0-10)
 *   [7:7]   vcm_active (1 bit, 1 = VCM superframe active)
 *   [8:12]  vcm_slot   (5 bits, current slot in superframe, 0-31)
 *   [13:17] vcm_total  (5 bits, total slots in superframe, 1-32)
 *   [18:23] reserved   (6 bits, zero)
 *   [24:31] crc8       (8 bits, CRC-8 over bits 0-23)
 *
 * The 32-bit payload is FEC-coded to 112 bits (seven RM(1,4) blocks) and
 * that codeword is repeated to fill the data subcarriers; the receiver
 * soft-combines the repeats. (The legacy encodePLS/decodePLS below used a
 * plain CRC-8 + 2x byte repetition with no coding gain — kept for tests.)
 *
 * === Variable Coding and Modulation (VCM) ===
 *
 * Broadcast VCM divides transmission into "superframes" of N slots.
 * Each slot can carry a different ModCod for multi-tier service:
 *
 *   Example superframe (4 slots):
 *     Slot 0: QPSK 1/2   → PLP0 (robust mono audio)
 *     Slot 1: QPSK 1/2   → PLP0 (continued)
 *     Slot 2: QAM64 3/4  → PLP1 (HD stereo enhancement)
 *     Slot 3: QAM64 3/4  → PLP1 (continued)
 *
 * The PLS block in each frame's preamble signals which slot we're in,
 * so the RX can:
 *   - Always decode the robust PLP (ignoring slots it can't decode)
 *   - Optionally decode the enhancement PLP when SNR permits
 *
 * This is analogous to DVB-T2 PLPs or ATSC 3.0 subframes.
 *
 * Reference: ETSI EN 302 755 (DVB-T2), ATSC A/322 (ATSC 3.0)
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <array>
#include <string>
#include <cstring>

namespace gw {

// =========================================================================
// PLS Block
// =========================================================================

/** Physical Layer Signaling data (decoded from PLS OFDM symbol) */
struct PLSBlock {
    Modulation modulation  = Modulation::QPSK;
    FECRate    fec_rate     = FECRate::Rate_1_2;
    bool       vcm_active   = false;
    uint8_t    vcm_slot     = 0;     ///< Current slot index (0-based)
    uint8_t    vcm_total    = 1;     ///< Total slots in superframe (1-32)

    bool       valid        = false; ///< Set by decoder on successful parse
};

/** Encode a PLS block to raw bits (64 bits = 8 bytes, with redundancy).
 *  Legacy byte/repetition codec — superseded in the engine by the
 *  Reed-Muller coded path (encodePLSCoded/decodePLSSoft); retained for the
 *  unit tests. */
void encodePLS(const PLSBlock& pls, std::vector<uint8_t>& bits_out);

/** Decode a PLS block from raw bits (legacy byte/repetition codec).
 *  @return true if CRC valid (dual-copy agreement checked) */
bool decodePLS(const std::vector<uint8_t>& bits_in, PLSBlock& pls);

/** CRC-8 for PLS (polynomial 0x07, init 0xFF) */
uint8_t plsCRC8(const uint8_t* data, size_t bits);

// =========================================================================
// Coded PLS (first-order Reed-Muller FEC + soft-combining)
//
// The bootstrap PLS must survive at the SNR cliff. The 32-bit payload
// (the same mod/fec/vcm/CRC-8 layout as encodePLS) is protected by seven
// first-order Reed-Muller RM(1,4) = [16,5,8] blocks (112 coded bits),
// soft-decoded near-ML by a 16-point fast Walsh-Hadamard transform. The
// coded word is mapped to genuine BPSK (independent of the data modcod) and
// repeated to fill the symbol's data subcarriers; the receiver soft-combines
// the repeats (LLR sum) before decoding, so coding gain AND combining gain
// both apply. Replaces the previous "CRC-8 + 2x hard repetition" which had
// neither (audit #26).
// =========================================================================

/// Number of RM(1,4) blocks and total coded bits for one PLS codeword.
static constexpr size_t PLS_RM_BLOCKS  = 7;
static constexpr size_t PLS_CODED_BITS = PLS_RM_BLOCKS * 16;  // 112

/** Encode a PLS block to `PLS_CODED_BITS` coded bits (one codeword, each
 *  element 0/1), ready for BPSK mapping. The caller repeats this codeword to
 *  fill the available data subcarriers. */
void encodePLSCoded(const PLSBlock& pls, std::vector<uint8_t>& coded_bits);

/** Decode a coded PLS from per-data-subcarrier soft values (BPSK convention:
 *  value > 0 ⇒ bit 0). `soft` holds the whole symbol's data subcarriers; the
 *  decoder soft-combines floor(soft.size()/PLS_CODED_BITS) repeated copies,
 *  RM-decodes the seven blocks, and verifies CRC-8.
 *  @return true if the CRC check passes. */
bool decodePLSSoft(const std::vector<float>& soft, PLSBlock& pls);

// =========================================================================
// VCM Slot Configuration
// =========================================================================

struct VCMSlot {
    Modulation modulation = Modulation::QPSK;
    FECRate    fec_rate    = FECRate::Rate_1_2;
    uint8_t    plp_id      = 0;       ///< Physical Layer Pipe ID (service tier)
    char       label[32]   = "Default";

    void setLabel(const char* l) {
        std::strncpy(label, l, sizeof(label) - 1);
        label[sizeof(label) - 1] = '\0';
    }
    std::string getLabel() const { return std::string(label); }
};

// =========================================================================
// VCM Schedule (Superframe definition)
// =========================================================================

static constexpr size_t VCM_MAX_SLOTS = 32;

struct VCMSchedule {
    bool     enabled    = false;
    uint8_t  num_slots  = 1;       ///< Number of active slots (1-32)
    std::array<VCMSlot, VCM_MAX_SLOTS> entries;

    /** Get the ModCod for a given frame number within the superframe */
    const VCMSlot& slotForFrame(uint32_t frame_number) const {
        if (!enabled || num_slots == 0) {
            static const VCMSlot default_slot{};
            return default_slot;
        }
        uint8_t idx = static_cast<uint8_t>(frame_number % num_slots);
        return entries[idx];
    }

    /** Build a PLS block for the given frame number */
    PLSBlock plsForFrame(uint32_t frame_number) const {
        PLSBlock pls;
        if (!enabled || num_slots == 0) {
            // Uniform (non-VCM) — just signal current modcod
            pls.vcm_active = false;
            pls.vcm_slot = 0;
            pls.vcm_total = 1;
            return pls;
        }
        uint8_t idx = static_cast<uint8_t>(frame_number % num_slots);
        const auto& s = entries[idx];
        pls.modulation = s.modulation;
        pls.fec_rate   = s.fec_rate;
        pls.vcm_active = true;
        pls.vcm_slot   = idx;
        pls.vcm_total  = num_slots;
        return pls;
    }
};

// =========================================================================
// Factory presets for common VCM configurations
// =========================================================================

/** Create a 2-tier M/S stereo VCM schedule.
 *  PLP0 (slots 0..n_robust-1): robust mono at given modcod
 *  PLP1 (slots n_robust..total-1): enhancement stereo at given modcod */
inline VCMSchedule createStereoVCM(
        Modulation robust_mod, FECRate robust_fec,
        Modulation enhance_mod, FECRate enhance_fec,
        uint8_t n_robust = 2, uint8_t n_enhance = 2) {
    VCMSchedule sched;
    sched.enabled = true;
    sched.num_slots = n_robust + n_enhance;

    for (uint8_t i = 0; i < n_robust; ++i) {
        sched.entries[i].modulation = robust_mod;
        sched.entries[i].fec_rate   = robust_fec;
        sched.entries[i].plp_id     = 0;
        sched.entries[i].setLabel("Mono (Robust)");
    }
    for (uint8_t i = 0; i < n_enhance; ++i) {
        uint8_t idx = n_robust + i;
        sched.entries[idx].modulation = enhance_mod;
        sched.entries[idx].fec_rate   = enhance_fec;
        sched.entries[idx].plp_id     = 1;
        sched.entries[idx].setLabel("Stereo (Enhance)");
    }
    return sched;
}

// =========================================================================
// ModCod Detector (RX side)
// =========================================================================

/**
 * RX-side auto-configuration from PLS.
 *
 * The detector maintains the last-known ModCod and signals when a
 * reconfiguration is needed. It validates PLS CRC before applying
 * changes and requires N consecutive matching PLS blocks before
 * triggering a change (to avoid glitches from corrupted PLS).
 */
class ModCodDetector {
public:
    explicit ModCodDetector(uint8_t confirm_count = 2);
    ~ModCodDetector();

    /**
     * Feed a decoded PLS block.
     * @return true if the ModCod changed (TX reconfigured or new VCM slot)
     */
    bool feed(const PLSBlock& pls);

    /** Get the current active ModCod */
    Modulation currentModulation() const { return current_mod_; }
    FECRate    currentFECRate()    const { return current_fec_; }

    /** Is VCM active? */
    bool vcmActive()    const { return vcm_active_; }
    uint8_t vcmSlot()   const { return vcm_slot_; }
    uint8_t vcmTotal()  const { return vcm_total_; }

    /** Has the ModCod changed since last acknowledged? */
    bool hasChanged()  const { return changed_; }
    void acknowledge()       { changed_ = false; }

    /** How many consecutive matching PLS blocks the detector has seen for
     *  the currently-pending ModCod. 0 means a new candidate just appeared;
     *  reaching `confirm_count_` triggers a real switch. Surface for the
     *  PLS status widget so users can see the lock-in confidence. */
    uint8_t confirmationCount() const { return match_count_; }
    uint8_t confirmationThreshold() const { return confirm_count_; }

    /** Reset to initial state */
    void reset();

private:
    uint8_t    confirm_count_;  ///< Consecutive matching PLS needed

    Modulation current_mod_  = Modulation::QPSK;
    FECRate    current_fec_  = FECRate::Rate_1_2;
    bool       vcm_active_   = false;
    uint8_t    vcm_slot_     = 0;
    uint8_t    vcm_total_    = 1;
    bool       changed_      = false;

    // Confirmation tracking
    Modulation pending_mod_  = Modulation::QPSK;
    FECRate    pending_fec_  = FECRate::Rate_1_2;
    uint8_t    match_count_  = 0;
    bool       initialized_  = false;
};

} // namespace gw
