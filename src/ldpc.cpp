/**
 * @file ldpc.cpp
 * @brief LDPC codec implementation
 *
 * Matrix construction:
 *   QC-LDPC with base matrix B (m_b × n_b). Each entry B[i][j] is either
 *   -1 (zero block) or a circulant shift value 0..q-1. The expanded H has
 *   dimensions (m_b*q) × (n_b*q).
 *
 * Encoding (IRA structure):
 *   The parity portion of H has staircase structure:
 *     H_parity = [I; I+shift(1); I+shift(2); ...]  (lower bidiagonal)
 *   This gives O(n) encoding: p[0] = Σ(info contributions to check 0),
 *   p[i] = p[i-1] XOR Σ(info contributions to check i).
 *
 * Decoding (Normalized Min-Sum BP):
 *   Check node: sign-product × min-magnitude × normalization_factor
 *   Variable node: channel LLR + sum of incoming check messages
 *   Early termination when all parity checks pass.
 */

#include "ldpc.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>
#include <climits>

namespace gw {

// =========================================================================
// Bit manipulation helpers
// =========================================================================

namespace {

inline bool getBit(const uint8_t* d, size_t i) {
    return (d[i >> 3] >> (7 - (i & 7))) & 1;
}

inline void setBit(uint8_t* d, size_t i, bool v) {
    if (v) d[i >> 3] |=  (uint8_t(1) << (7 - (i & 7)));
    else   d[i >> 3] &= ~(uint8_t(1) << (7 - (i & 7)));
}

// Deterministic PRNG for matrix construction
struct PRNG {
    uint32_t state;
    explicit PRNG(uint32_t seed) : state(seed) {}
    uint32_t next() {
        state = state * 1103515245u + 12345u;
        return state;
    }
    uint32_t nextRange(uint32_t max) {
        return next() % max;
    }
};

} // anonymous

// =========================================================================
// Base Matrix Definitions
//
// Format: base_matrix[check_row][var_col] = circulant_shift or -1
// Columns 0..k_base-1 are information, k_base..n_base-1 are parity.
// Parity columns have staircase structure (enforced in expansion).
//
// These base matrices are designed for girth >= 6 with variable node
// degree 3-4 (information) and degree 2 (parity, from staircase).
// =========================================================================

namespace {

struct BaseMatrixDef {
    size_t m_base;  // check rows
    size_t k_base;  // info columns
    size_t n_base;  // total columns (k_base + m_base for IRA)

    // info_connections[check][col] = shift value, or -1 if absent
    // Only info columns; parity columns are always staircase.
    std::vector<std::vector<int>> info_shifts;
};

// Generate a base matrix with good properties using PEG-like deterministic construction
BaseMatrixDef generateBaseMatrix(size_t m_base, size_t k_base, uint32_t seed,
                                 size_t q) {
    BaseMatrixDef B;
    B.m_base = m_base;
    B.k_base = k_base;
    B.n_base = k_base + m_base;
    // Circulant shifts are drawn from [0, q) so the full lifting space is
    // used. Previously they were drawn from a hardcoded [0, 90), so for the
    // Normal block (q=360) only 1/4 of the shift range was exercised,
    // reducing girth/cycle diversity for the n=8640 code. (#74)
    const uint32_t shift_mod = static_cast<uint32_t>(q > 0 ? q : 90);

    // Target column weight for info columns: ~3-4 connections per column
    size_t target_col_weight = std::min(size_t(4), m_base);

    B.info_shifts.resize(m_base, std::vector<int>(k_base, -1));

    PRNG rng(seed);

    // For each info column, connect it to target_col_weight check nodes
    for (size_t col = 0; col < k_base; ++col) {
        // Track which rows this column is connected to
        std::vector<bool> connected(m_base, false);
        size_t count = 0;

        // First connection: distribute evenly
        size_t first_row = col % m_base;
        B.info_shifts[first_row][col] = static_cast<int>(rng.nextRange(shift_mod));
        connected[first_row] = true;
        count++;

        // Additional connections: pick rows with lowest current degree
        while (count < target_col_weight) {
            // Find row with minimum degree (not already connected)
            size_t best_row = 0;
            int best_degree = static_cast<int>(k_base) + 1;
            size_t start = rng.nextRange(static_cast<uint32_t>(m_base));

            for (size_t ri = 0; ri < m_base; ++ri) {
                size_t r = (start + ri) % m_base;
                if (connected[r]) continue;

                int deg = 0;
                for (size_t c = 0; c < k_base; ++c) {
                    if (B.info_shifts[r][c] >= 0) deg++;
                }
                if (deg < best_degree) {
                    best_degree = deg;
                    best_row = r;
                }
            }

            if (!connected[best_row]) {
                // Bug fix: original code wrote `info_shifts[best_row][best_row]`
                // first (out-of-bounds for the column index when best_row >=
                // k_base — heap corruption on rate 1/4 / 1/3 / 2/5 etc.)
                // Only the second write (the legit `[best_row][col]`) is
                // needed; drop the buggy first one.
                B.info_shifts[best_row][col] =
                    static_cast<int>(rng.nextRange(shift_mod));
                connected[best_row] = true;
                count++;
            } else {
                break; // all rows connected
            }
        }
    }

    return B;
}

} // anonymous

// =========================================================================
// Protograph (girth-conditioned QC-LDPC) construction
//
// The information part is built in two stages:
//   1. buildInfoConnectivity — decide WHICH base cells are non-zero,
//      balancing check-row degrees (same philosophy as the legacy
//      generator). No shift values yet.
//   2. assignGirthShifts    — choose each non-zero cell's circulant shift
//      greedily so the LIFTED Tanner graph is free of 4-cycles
//      (girth >= 6). The IRA staircase parity (fixed shifts) is included
//      in the cycle bookkeeping so info<->parity 4-cycles are avoided too.
//
// A lifted 4-cycle exists between base columns a,b iff two rows r1,r2 both
// connect a and b with equal shift differences:
//   shift(r1,a) - shift(r1,b)  ==  shift(r2,a) - shift(r2,b)  (mod q).
// We therefore track, per column pair, the multiset of per-row shift
// differences and reject any shift that would repeat a difference.
// =========================================================================

namespace {

inline int modq(long long x, long long q) {
    long long r = x % q;
    return static_cast<int>(r < 0 ? r + q : r);
}

// present[br][bc] = true where info cell (br,bc) is non-zero.
std::vector<std::vector<bool>> buildInfoConnectivity(
        size_t m_base, size_t k_base, size_t col_weight, uint32_t seed) {
    std::vector<std::vector<bool>> present(m_base, std::vector<bool>(k_base, false));
    std::vector<int> row_deg(m_base, 0);
    PRNG rng(seed);

    const size_t target = std::min(col_weight, m_base);
    for (size_t col = 0; col < k_base; ++col) {
        size_t cnt = 0;
        // First connection: spread evenly across rows by column index.
        size_t first_row = col % m_base;
        present[first_row][col] = true;
        row_deg[first_row]++;
        cnt++;
        // Remaining connections: lowest-degree not-yet-connected row,
        // with a randomized start so ties break differently per column.
        while (cnt < target) {
            size_t start = rng.nextRange(static_cast<uint32_t>(m_base));
            size_t best = m_base;
            int best_deg = INT_MAX;
            for (size_t ri = 0; ri < m_base; ++ri) {
                size_t r = (start + ri) % m_base;
                if (present[r][col]) continue;
                if (row_deg[r] < best_deg) { best_deg = row_deg[r]; best = r; }
            }
            if (best == m_base) break;
            present[best][col] = true;
            row_deg[best]++;
            cnt++;
        }
    }
    return present;
}

// Returns info_shifts[br][bc] (shift value, or -1 if cell absent), with
// shifts chosen for girth >= 6 against the info cells AND the fixed IRA
// staircase parity cells.
std::vector<std::vector<int>> assignGirthShifts(
        const std::vector<std::vector<bool>>& present,
        size_t m_base, size_t k_base, size_t q, uint32_t seed) {
    const size_t n_base = k_base + m_base;

    // pair_diffs[(a,b)] (a<b, full-base column indices) = list of per-shared-
    // row shift differences (shift(r,a)-shift(r,b)) mod q seen so far.
    std::map<std::pair<uint32_t, uint32_t>, std::vector<int>> pair_diffs;
    // row_entries[r] = (full-base column, shift) cells already placed in row r.
    std::vector<std::vector<std::pair<uint32_t, int>>> row_entries(m_base);

    auto orientedDiff = [&](uint32_t cA, int sA, uint32_t cB, int sB)
            -> std::pair<std::pair<uint32_t, uint32_t>, int> {
        if (cA < cB) return {{cA, cB}, modq((long long)sA - sB, (long long)q)};
        else         return {{cB, cA}, modq((long long)sB - sA, (long long)q)};
    };
    // Number of 4-cycles a placement at (row,col,s) would create.
    auto conflicts = [&](uint32_t row, uint32_t col, int s) -> int {
        int c = 0;
        for (const auto& e : row_entries[row]) {
            auto od = orientedDiff(col, s, e.first, e.second);
            auto it = pair_diffs.find(od.first);
            if (it != pair_diffs.end() &&
                std::find(it->second.begin(), it->second.end(), od.second)
                    != it->second.end())
                ++c;
        }
        return c;
    };
    auto place = [&](uint32_t row, uint32_t col, int s) {
        for (const auto& e : row_entries[row]) {
            auto od = orientedDiff(col, s, e.first, e.second);
            pair_diffs[od.first].push_back(od.second);
        }
        row_entries[row].push_back({col, s});
    };

    // 1. Place the fixed IRA staircase parity cells first (col = k_base+p):
    //    diagonal (row p) shift 0, sub-diagonal (row p, prev parity col) shift 1.
    for (size_t p = 0; p < m_base; ++p) {
        place(static_cast<uint32_t>(p),
              static_cast<uint32_t>(k_base + p), 0);
        if (p > 0)
            place(static_cast<uint32_t>(p),
                  static_cast<uint32_t>(k_base + (p - 1)), 1);
    }
    (void)n_base;

    // 2. Greedily assign info shifts.
    std::vector<std::vector<int>> info_shifts(
        m_base, std::vector<int>(k_base, -1));
    PRNG rng(seed);
    std::vector<int> cand(q);
    std::iota(cand.begin(), cand.end(), 0);

    for (size_t bc = 0; bc < k_base; ++bc) {
        for (size_t br = 0; br < m_base; ++br) {
            if (!present[br][bc]) continue;
            // Deterministic Fisher-Yates shuffle of candidate shifts so the
            // search order is well-mixed but reproducible.
            for (size_t i = q; i > 1; --i) {
                size_t j = rng.nextRange(static_cast<uint32_t>(i));
                std::swap(cand[i - 1], cand[j]);
            }
            int best_shift = cand[0];
            int best_conf = INT_MAX;
            for (int s : cand) {
                int c = conflicts(static_cast<uint32_t>(br),
                                  static_cast<uint32_t>(bc), s);
                if (c < best_conf) { best_conf = c; best_shift = s; }
                if (c == 0) break;  // girth-6 preserved; take it
            }
            info_shifts[br][bc] = best_shift;
            place(static_cast<uint32_t>(br),
                  static_cast<uint32_t>(bc), best_shift);
        }
    }
    return info_shifts;
}

// Expand a base info-shift matrix + the fixed IRA staircase into the full
// sparse lifted H. Shared by the Legacy and Protograph builders.
std::shared_ptr<LDPCMatrix> expandBase(
        size_t m_base, size_t k_base, size_t q,
        const std::vector<std::vector<int>>& info_shifts) {
    const size_t n_base = k_base + m_base;

    auto H = std::make_shared<LDPCMatrix>();
    H->q = q;
    H->n = n_base * q;
    H->k = k_base * q;
    H->m = m_base * q;
    H->rows.resize(H->m);
    H->cols.resize(H->n);

    // Information columns: circulant blocks from the base shifts.
    for (size_t br = 0; br < m_base; ++br) {
        for (size_t bc = 0; bc < k_base; ++bc) {
            if (info_shifts[br][bc] < 0) continue;
            int shift = info_shifts[br][bc] % static_cast<int>(q);
            for (size_t i = 0; i < q; ++i) {
                uint32_t row = static_cast<uint32_t>(br * q + i);
                uint32_t col = static_cast<uint32_t>(
                    bc * q + ((i + static_cast<size_t>(shift)) % q));
                H->rows[row].push_back(col);
                H->cols[col].push_back(row);
            }
        }
    }

    // Parity columns: staircase (IRA) — diagonal identity + shift-1 sub-diagonal.
    size_t parity_col_start = k_base * q;
    for (size_t br = 0; br < m_base; ++br) {
        for (size_t i = 0; i < q; ++i) {
            uint32_t row = static_cast<uint32_t>(br * q + i);
            uint32_t col = static_cast<uint32_t>(parity_col_start + br * q + i);
            H->rows[row].push_back(col);
            H->cols[col].push_back(row);
        }
        if (br > 0) {
            for (size_t i = 0; i < q; ++i) {
                uint32_t row = static_cast<uint32_t>(br * q + i);
                uint32_t col = static_cast<uint32_t>(
                    parity_col_start + (br - 1) * q + ((i + 1) % q));
                H->rows[row].push_back(col);
                H->cols[col].push_back(row);
            }
        }
    }
    return H;
}

// Legacy base dimensions + deterministic seed for a rate.
struct LegacyParams { size_t m_base; size_t k_base; uint32_t seed; };
LegacyParams legacyParams(FECRate rate) {
    switch (rate) {
        case FECRate::Rate_1_4:  return {18, 6,  0x1A2B3C4Du};
        case FECRate::Rate_1_3:  return {16, 8,  0x2B3C4D5Eu};
        case FECRate::Rate_2_5:  return {15, 10, 0x3C4D5E6Fu};
        case FECRate::Rate_1_2:  return {12, 12, 0x4D5E6F70u};
        case FECRate::Rate_3_5:  return {10, 15, 0x5E6F7081u};
        case FECRate::Rate_2_3:  return {8,  16, 0x6F708192u};
        case FECRate::Rate_3_4:  return {6,  18, 0x708192A3u};
        case FECRate::Rate_4_5:  return {5,  20, 0x8192A3B4u};
        case FECRate::Rate_5_6:  return {4,  20, 0x92A3B4C5u};
        case FECRate::Rate_8_9:  return {3,  24, 0xA3B4C5D6u};
        case FECRate::Rate_9_10: return {3,  27, 0xB4C5D6E7u};
        default:                 return {12, 12, 0x4D5E6F70u};
    }
}

// Protograph base-lifting factor g: the high rates whose legacy base is too
// short (m_base <= 5) get a TALLER base (m_base*g rows) with a smaller
// circulant (q/g) so the lifted (n,k,m) are unchanged but there is room to
// kill 4-cycles. g must divide q (90 and 360 both divide by 2 and 3).
size_t protographScale(FECRate rate) {
    switch (rate) {
        case FECRate::Rate_9_10: return 3;  // m_base 3 -> 9,  q 90->30 / 360->120
        case FECRate::Rate_8_9:  return 3;  // m_base 3 -> 9
        case FECRate::Rate_4_5:  return 2;  // m_base 5 -> 10, q 90->45 / 360->180
        case FECRate::Rate_5_6:  return 2;  // m_base 4 -> 8
        default:                 return 1;  // base already tall enough
    }
}

} // anonymous

// =========================================================================
// Build Full LDPC Matrix from Base Matrix
// =========================================================================

std::shared_ptr<LDPCMatrix> buildLDPCMatrix(FECRate rate, LDPCBlockSize blk,
                                            LDPCConstruction construction) {
    size_t q_full;
    switch (blk) {
        case LDPCBlockSize::Short:  q_full = 90;  break;
        case LDPCBlockSize::Normal: q_full = 360; break;
        default: q_full = 90;
    }

    LegacyParams lp = legacyParams(rate);

    if (construction == LDPCConstruction::Legacy) {
        BaseMatrixDef B = generateBaseMatrix(lp.m_base, lp.k_base, lp.seed, q_full);
        return expandBase(lp.m_base, lp.k_base, q_full, B.info_shifts);
    }

    // Protograph: lift the base by g (q shrinks so n,k,m are invariant).
    size_t g = protographScale(rate);
    if (g == 0 || (q_full % g) != 0) g = 1;  // safety: keep q integer
    size_t m_base = lp.m_base * g;
    size_t k_base = lp.k_base * g;
    size_t q      = q_full / g;
    // Info variable-node degree. The taller high-rate bases (m_base >= 8)
    // support degree 4, which strengthens variable nodes and breaks the
    // small min-sum trapping sets that degree-3 high-rate codes fall into
    // (a degree-3 9/10 protograph stalls at a 5-check near-codeword under
    // the dense QAM256 LLR profile). Low rates (g=1) keep their legacy degree.
    size_t col_weight = std::min<size_t>(4, m_base);

    auto present = buildInfoConnectivity(m_base, k_base, col_weight, lp.seed);
    auto info_shifts = assignGirthShifts(present, m_base, k_base, q,
                                         lp.seed ^ 0x9E3779B9u);
    return expandBase(m_base, k_base, q, info_shifts);
}

// =========================================================================
// 4-cycle (girth-4) counter — diagnostic / test helper
// =========================================================================

size_t countLDPCShortCycles(const LDPCMatrix& H) {
    // For each check, every unordered pair of its variables co-occurs once;
    // a variable pair sharing `s` checks contributes C(s,2) four-cycles.
    std::unordered_map<uint64_t, uint32_t> co;
    for (size_t c = 0; c < H.m; ++c) {
        const auto& vars = H.rows[c];
        for (size_t a = 0; a < vars.size(); ++a) {
            for (size_t b = a + 1; b < vars.size(); ++b) {
                uint32_t v1 = vars[a], v2 = vars[b];
                if (v1 > v2) std::swap(v1, v2);
                uint64_t key = (static_cast<uint64_t>(v1) << 32) | v2;
                ++co[key];
            }
        }
    }
    size_t cycles = 0;
    for (const auto& kv : co) {
        uint64_t s = kv.second;
        cycles += static_cast<size_t>(s * (s - 1) / 2);
    }
    return cycles;
}

// =========================================================================
// LDPC Encoder — IRA staircase encoding
// =========================================================================

LDPCEncoder::LDPCEncoder(FECRate rate, LDPCBlockSize blk,
                         LDPCConstruction construction) {
    H_ = buildLDPCMatrix(rate, blk, construction);
}

bool LDPCEncoder::encode(const uint8_t* info, uint8_t* cw) const {
    const size_t k = H_->k;
    const size_t n = H_->n;
    const size_t m = H_->m;

    // Copy info bits to codeword (systematic)
    size_t k_bytes = (k + 7) / 8;
    std::memcpy(cw, info, k_bytes);

    // Clear parity portion
    size_t n_bytes = (n + 7) / 8;
    std::memset(cw + k_bytes, 0, n_bytes - k_bytes);

    // Compute parity bits using IRA accumulate structure
    // For each check equation i: p[i] = p[i-1] XOR (sum of info bits in check i)
    // Since H_parity is staircase, this is a simple accumulator.

    // First: compute syndrome of info bits for each check
    std::vector<uint8_t> syndrome(m, 0);
    for (size_t check = 0; check < m; ++check) {
        uint8_t s = 0;
        for (uint32_t col : H_->rows[check]) {
            if (col < static_cast<uint32_t>(k)) {
                // Info bit
                s ^= getBit(info, col) ? 1 : 0;
            }
        }
        syndrome[check] = s;
    }

    // Now solve for parity bits using staircase:
    // Row 0: p[0] = syndrome[0]
    // Row i: p[i] = syndrome[i] XOR p[i-1] (due to sub-diagonal)
    //
    // But the actual staircase is in circulant blocks, so we process
    // block by block within the q-expanded structure.
    //
    // Simpler approach: direct back-substitution on the staircase.
    // Since H_parity is lower bidiagonal, we can solve row by row.

    // For the expanded IRA structure, iterate checks in order.
    // Each check connects to parity bits. The staircase means each check
    // has its own parity bit + the previous block's parity bit.
    // We solve for parity[i] given that all earlier parity bits are known.

    for (size_t check = 0; check < m; ++check) {
        // Compute partial syndrome from all KNOWN bits (info + earlier parity)
        uint8_t s = 0;
        uint32_t unknown_parity_col = UINT32_MAX;

        for (uint32_t col : H_->rows[check]) {
            if (col < static_cast<uint32_t>(k)) {
                // Info bit — already in codeword
                s ^= getBit(cw, col) ? 1 : 0;
            } else {
                // Parity bit
                size_t parity_idx = col - static_cast<uint32_t>(k);
                if (parity_idx <= check) {
                    // Already computed parity bit, or the current one
                    if (parity_idx == check) {
                        unknown_parity_col = col;
                    } else {
                        s ^= getBit(cw, col) ? 1 : 0;
                    }
                }
            }
        }

        // Set the unknown parity bit to satisfy this check equation
        if (unknown_parity_col != UINT32_MAX) {
            setBit(cw, unknown_parity_col, s != 0);
        }
    }

    return true;
}

// =========================================================================
// LDPC Decoder — Normalized Min-Sum BP
// =========================================================================

LDPCDecoder::LDPCDecoder(FECRate rate, LDPCBlockSize blk, size_t max_iter,
                         LDPCConstruction construction)
    : max_iter_(max_iter), norm_factor_(0.75f)
{
    H_ = buildLDPCMatrix(rate, blk, construction);

    const size_t n = H_->n;
    const size_t m = H_->m;

    posterior_.resize(n);

    // c2v message store for the layered min-sum schedule (read each
    // iteration to back out the extrinsic v2c value).
    msg_c2v_.resize(m);
    for (size_t c = 0; c < m; ++c) {
        msg_c2v_[c].resize(H_->rows[c].size(), 0.f);
    }

    // NOTE: the former msg_v2c_ / c2v_pos_ / v2c_pos_ edge-index maps were
    // removed — the layered decodeFull() schedule indexes rows/cols and
    // posterior_ directly and never read them (the header even noted
    // "msg_v2c_ is unused in this schedule"). They cost ~250 KB of dead
    // vectors plus an O(edges×deg) build at Rate_9_10 Normal for nothing,
    // and the header's "halves wall-clock decode time" claim was false.

    hard_bits_.resize((n + 7) / 8, 0);
}

void LDPCDecoder::hardDecision(const std::vector<float>& llr,
                                std::vector<uint8_t>& hard) const {
    size_t n = llr.size();
    hard.assign((n + 7) / 8, 0);
    for (size_t i = 0; i < n; ++i) {
        if (llr[i] < 0.f) {
            setBit(hard.data(), i, true);
        }
    }
}

bool LDPCDecoder::checkParity(const std::vector<uint8_t>& hard) const {
    for (size_t c = 0; c < H_->m; ++c) {
        uint8_t parity = 0;
        for (uint32_t v : H_->rows[c]) {
            parity ^= getBit(hard.data(), v) ? 1 : 0;
        }
        if (parity) return false;
    }
    return true;
}

LDPCDecodeResult LDPCDecoder::decodeFull(const float* llr_in, uint8_t* hard_out) {
    LDPCDecodeResult result;
    const size_t n = H_->n;
    const size_t m = H_->m;

    // Layered (row-by-row) min-sum belief propagation. Each iteration
    // visits each check once; within a check, c2v messages are computed
    // from the current posteriors (= channel LLR + accumulated c2v),
    // then the posteriors are updated atomically before moving to the
    // next check. This "use newest information immediately" schedule
    // typically converges in ~half the iterations of the flooding
    // schedule, with identical fixed-point output.
    //
    // Storage: msg_c2v_[c][i] stores the LAST c2v sent from check c on
    // edge i. v2c is derived implicitly as (posterior[v] - last_c2v).
    // msg_v2c_ is unused in this schedule but kept allocated for ABI
    // stability (decodePosterior etc. don't touch it).
    for (size_t v = 0; v < n; ++v) posterior_[v] = llr_in[v];
    for (size_t c = 0; c < m; ++c) {
        std::fill(msg_c2v_[c].begin(), msg_c2v_[c].end(), 0.f);
    }

    // Per-row scratch for the gathered v2c values. Allocated once
    // outside the iteration loop so we don't churn the allocator.
    std::vector<float> v2c_scratch;
    v2c_scratch.reserve(32);

    for (size_t iter = 0; iter < max_iter_; ++iter) {
        for (size_t c = 0; c < m; ++c) {
            const auto& row = H_->rows[c];
            size_t deg = row.size();
            if (deg == 0) continue;
            v2c_scratch.resize(deg);

            // Single pass: compute v2c[i] = posterior[row[i]] − old_c2v,
            // track sign product and the two smallest magnitudes.
            float min1 = std::numeric_limits<float>::max();
            float min2 = std::numeric_limits<float>::max();
            size_t min1_idx = 0;
            int total_sign = 1;
            float* old_c2v = msg_c2v_[c].data();
            for (size_t i = 0; i < deg; ++i) {
                uint32_t v = row[i];
                float v2c = posterior_[v] - old_c2v[i];
                v2c_scratch[i] = v2c;
                int s = (v2c >= 0.f) ? 1 : -1;
                total_sign *= s;
                float mag = std::abs(v2c);
                if (mag < min1) {
                    min2 = min1;
                    min1 = mag;
                    min1_idx = i;
                } else if (mag < min2) {
                    min2 = mag;
                }
            }

            // Output pass: compute new c2v[i] and apply the delta to
            // posterior[row[i]] in one step. The delta replaces the old
            // contribution of this check on variable row[i].
            for (size_t i = 0; i < deg; ++i) {
                float v2c = v2c_scratch[i];
                int in_sign = (v2c >= 0.f) ? 1 : -1;
                int out_sign = total_sign * in_sign;
                float out_mag = (i == min1_idx) ? min2 : min1;
                float new_c2v = norm_factor_ *
                                static_cast<float>(out_sign) * out_mag;
                if (new_c2v >  50.f) new_c2v =  50.f;
                if (new_c2v < -50.f) new_c2v = -50.f;

                uint32_t v = row[i];
                posterior_[v] += (new_c2v - old_c2v[i]);
                old_c2v[i] = new_c2v;
            }
        }

        // ---- Early termination check ----
        // Layered LDPC converges much faster than flooding (often in
        // 2-3 iterations on clean channels), so we check every iteration
        // instead of every 5. The parity check is O(m·avg_deg) which is
        // small relative to a full iteration's work and pays back
        // immediately when easy cases converge fast.
        hardDecision(posterior_, hard_bits_);
        if (checkParity(hard_bits_)) {
            result.converged = true;
            result.iterations = iter + 1;
            break;
        }
        result.iterations = iter + 1;
    }

    // Copy hard decision to output
    hardDecision(posterior_, hard_bits_);
    size_t n_bytes = (n + 7) / 8;
    std::memcpy(hard_out, hard_bits_.data(), n_bytes);

    // Average confidence
    float sum_mag = 0.f;
    for (size_t i = 0; i < n; ++i) sum_mag += std::abs(posterior_[i]);
    result.avg_magnitude = sum_mag / static_cast<float>(n);

    return result;
}

LDPCDecodeResult LDPCDecoder::decode(const float* llr_in, uint8_t* info_out) {
    size_t n_bytes = (H_->n + 7) / 8;
    std::vector<uint8_t> full_cw(n_bytes, 0);

    auto result = decodeFull(llr_in, full_cw.data());

    // Extract systematic info bits (first k bits)
    size_t k_bytes = (H_->k + 7) / 8;
    std::memcpy(info_out, full_cw.data(), k_bytes);

    return result;
}

LDPCDecodeResult LDPCDecoder::decodePosterior(const float* llr_in,
                                                std::vector<float>& post_out) {
    // Run the standard decode loop, then snapshot the per-bit posterior LLRs.
    // The decoder already maintains `posterior_` as the running sum
    // channel_llr + sum(c2v). Calling decodeFull() leaves it populated with
    // the final-iteration values, which is exactly what we want for BICM-ID.
    size_t n = H_->n;
    size_t n_bytes = (n + 7) / 8;
    std::vector<uint8_t> hard(n_bytes, 0);
    auto result = decodeFull(llr_in, hard.data());
    post_out.assign(posterior_.begin(), posterior_.end());
    return result;
}

} // namespace gw
