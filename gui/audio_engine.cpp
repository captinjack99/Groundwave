/**
 * @file audio_engine.cpp
 * @brief Audio engine implementation
 *
 * Processing loop runs at ~10ms intervals in a dedicated thread.
 * Each tick:
 *   1. Sync configuration from AppState (under lock)
 *   2. If TX enabled: generate audio → encode → modulate → transmit
 *   3. In loopback: process loopback (optionally with AWGN)
 *   4. Receive baseband → accumulate → demodulate → decode
 *   5. Push measurements to AppState and spectrum data to DataBridge
 */

#include "audio_engine.hpp"
#include "../include/polyphase.hpp"
#include <QCoreApplication>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <set>

namespace gw {

// The AUDIO content rate — Opus codec, test-tone synthesis, decoded-stream
// playback/recording, and mic capture. Fixed at 48 kHz (a rate Opus
// accepts) and deliberately INDEPENDENT of the RF/OFDM sample rate: audio
// rides inside frames as bytes, so the modem's sample rate (96/192 kHz on
// the wideband presets) is free to differ. Coupling the codec to
// ofdm_p_.sample_rate made OpusAudioEncoder's ctor throw for every
// non-Opus RF rate — initDSP caught it, left the DSP null, and the engine
// sat idle: the shipped Ultra HD (96 k) and Broadcast (192 k) presets
// bricked the engine on selection.
constexpr uint32_t kAudioSampleRate = 48000;

// =========================================================================
// Constructor / Destructor
// =========================================================================

AudioEngine::AudioEngine(AppState& state, DataBridge& bridge, QObject* parent)
    : QObject(parent)
    , state_(state)
    , bridge_(bridge)
{
    // Move this object to the worker thread
    this->moveToThread(&thread_);
}

AudioEngine::~AudioEngine() {
    shutdown();
}

// =========================================================================
// Thread lifecycle
// =========================================================================

void AudioEngine::startup() {
    // Also gate on thread_.isRunning(): running_ is only set true inside the
    // QThread::started lambda, so a second startup() (e.g. repeated F9) in the
    // window between thread_.start() and the lambda would otherwise pass the
    // running_ check and double-init / create a second timer.
    if (running_.load() || thread_.isRunning()) return;
    shutdown_requested_.store(false);

    // CRITICAL: move this AudioEngine onto the engine thread BEFORE starting
    // it, so the QTimer (created with `this` as parent) and processTick
    // slot fire on the engine thread — not the GUI thread. Otherwise every
    // DSP tick blocks the UI for 5–50 ms, producing the "GUI hangs, briefly
    // updates, hangs again" pattern. Cross-thread signal connections to
    // GUI widgets become QueuedConnection automatically (Qt::AutoConnection
    // detects the thread mismatch).
    if (thread() != &thread_) {
        moveToThread(&thread_);
    }

    // Disconnect any prior connection to avoid duplicate init on restart
    disconnect(&thread_, &QThread::started, this, nullptr);

    // Connect thread started → init DSP + start timer. With AudioEngine
    // now living on `thread_`, this slot runs on the engine thread.
    connect(&thread_, &QThread::started, this, [this]() {
        // A shutdown() issued during the startup window (between
        // thread_.start() and this lambda) must prevent bring-up entirely.
        // Previously init proceeded, exec() then consumed the already-
        // pending quit() and exited instantly — leaving running_ == true
        // with live devices and a DEAD event loop: startup() refused
        // forever ("already running") and shutdown()'s queued teardown
        // could never execute. F10 (or closing the window) within the
        // startup second could wedge the engine for the session.
        if (shutdown_requested_.load()) return;
        initDSP();
        timer_ = new QTimer(this);
        timer_->setTimerType(Qt::PreciseTimer);
        connect(timer_, &QTimer::timeout, this, &AudioEngine::processTick);
        timer_->start(static_cast<int>(ecfg_.process_interval_ms));
        running_.store(true);
        emit engineStarted();
    });

    thread_.start();
}

void AudioEngine::shutdown() {
    if (!running_.load() && !thread_.isRunning()) return;

    shutdown_requested_.store(true);

    // ALWAYS route teardown through the engine thread's event queue (the
    // old code called thread_.quit() directly when running_ was still
    // false, racing the started-lambda — see the wedge note there). The
    // queued lambda runs after the started-lambda in every interleaving:
    // either a full teardown (init completed) or a bare quit (init was
    // skipped by shutdown_requested_).
    QMetaObject::invokeMethod(this, [this]() {
        if (timer_) {
            timer_->stop();
            delete timer_;
            timer_ = nullptr;
        }
        if (running_.load()) {
            teardownDSP();
            running_.store(false);
            emit engineStopped();
        }
        // Push this object back to the GUI thread from HERE — the owner
        // thread — as the final act before quitting. A push from the
        // owner is always legal; the old pull from shutdown() (GUI side,
        // after wait()) raced the thread's finish bookkeeping: on rapid
        // start→stop, isRunning() already read false while Qt's thread
        // data wasn't fully torn down, and the pull was refused with
        // "QObject::moveToThread: Current thread is not the object's
        // thread / Cannot move to target thread" (gui_walker reproduced
        // it on every Start→Stop pair). This lambda runs in every
        // shutdown interleaving — full teardown, or bare quit when the
        // started-lambda bailed on shutdown_requested_ — so the handoff
        // is unconditional.
        moveToThread(QCoreApplication::instance()->thread());
        thread_.quit();
    }, Qt::QueuedConnection);

    // Wait for the engine thread to FULLY stop before allowing destruction.
    // Proceeding while the engine thread is still inside teardownDSP()
    // (which may block on a device close) would be a use-after-free. If the
    // first wait times out, give teardown more time rather than racing
    // destruction.
    if (!thread_.wait(5000)) {
        thread_.wait(10000);
    }

    // Belt-and-braces: if the queued handoff could not run (it always
    // should — exec() drains the queue after the started-lambda), fall
    // back to the pull, which is legal once the thread has fully stopped.
    if (!thread_.isRunning() && thread() != QThread::currentThread()) {
        moveToThread(QThread::currentThread());
    }
}

// =========================================================================
// DSP initialization / teardown
// =========================================================================

void AudioEngine::initDSP() {
    // Read initial config from AppState
    syncConfig();

    try {
        // Soundcard modem — hardware or loopback
        if (ecfg_.use_hw_audio) {
            modem_p_.loopback = false;
            modem_p_.complex_loopback = false;
        } else {
            modem_p_.loopback = true;
            // Software loopback uses the complex (baseband) path: it stores
            // and returns the baseband verbatim — bit-exact and zero-delay —
            // so SW-loopback diagnostics isolate the modem from the analog
            // model. The passband real-IF round trip also works now that the
            // subcarrier allocation is DC-centered with guards at the band
            // edges (computeAllocation) — that's what HW/vcable mode and the
            // CLI's internal mode exercise — but it adds AGC settling, LPF
            // group delay, and band-edge rolloff that belong in passband
            // testing, not in the GUI's reference loopback.
            if (!modem_p_.complex_loopback) modem_p_.complex_loopback = true;
        }
        // Enable the raised-cosine TX power ramp so the carrier ramps up at
        // key-on and (via endTransmission()) down at key-off, instead of
        // hard-gating at full amplitude — eliminates on-air splatter for
        // FCC §73.319 mask compliance. ~5 ms at 48 kHz; scale with SR.
        modem_p_.enable_tx_ramp  = true;
        modem_p_.tx_ramp_samples = std::max<uint32_t>(
            64, ofdm_p_.sample_rate / 200);   // ~5 ms
        modem_ = std::make_unique<SoundcardModem>(modem_p_, ofdm_p_);
        prev_tx_enabled_ = false;

        // Hardware audio device (if enabled)
        hw_audio_.reset();
        if (ecfg_.use_hw_audio) {
            hw_audio_ = std::make_unique<HWAudioDevice>();
            bool hw_ok = false;
            if (hw_audio_->init()) {
                HWAudioConfig hacfg;
                hacfg.sample_rate     = ofdm_p_.sample_rate;
                hacfg.buffer_frames   = 512;
                hacfg.playback_device = ecfg_.playback_device;
                hacfg.capture_device  = ecfg_.capture_device;
                if (hw_audio_->start(modem_->txRing(), modem_->rxRing(), hacfg)) {
                    hw_ok = true;
                } else {
                    emit engineError("Hardware audio failed — falling back to loopback");
                }
            } else {
                emit engineError("Hardware audio init failed — falling back to loopback");
            }
            if (!hw_ok) {
                // Rebuild the modem in loopback mode. Without this the modem
                // is in HW-audio mode (loopback=false) but no HW reader/writer
                // exists — TX writes vanish into a ring nobody reads, RX gets
                // nothing. Worse, transmit() and receive() on a stale modem
                // can hit cleanup paths that crash. Do a clean rebuild.
                hw_audio_.reset();
                modem_p_.loopback = true;
                modem_ = std::make_unique<SoundcardModem>(modem_p_, ofdm_p_);
            }
        }

        // PAPR tone-reservation carves data-free carriers out of the
        // allocation; the modulator AND demodulator below must agree on that
        // split, so set the fraction on the OFDM params BEFORE building them.
        // Zero when PAPR-TR is off → no carve, byte-identical allocation.
        ofdm_p_.papr_reserve_fraction =
            ecfg_.papr.enabled ? ecfg_.papr.reserve_fraction : 0.f;

        // OFDM
        ofdm_mod_   = std::make_unique<OFDMModulator>(ofdm_p_);
        ofdm_demod_ = std::make_unique<OFDMDemodulator>(ofdm_p_);
        sync_        = std::make_unique<OFDMSynchronizer>(ofdm_p_);

        // Enable MMSE channel estimation if configured
        if (ecfg_.use_mmse) {
            ofdm_demod_->enableMMSE();
        }

        // AFC: phase-tracker PLL + sample-rate-offset estimator + the SFO
        // resampler loop. Driven by ModemConfig::enable_afc (Tuning panel).
        if (modem_p_.enable_afc) {
            ofdm_demod_->enablePhaseTracker();
            // ema_alpha 0.3: light slope smoothing; the resampler's PI loop
            // does the rest. (Lower alpha lags the ramping slope and
            // destabilises the loop; see sfo_test.cpp.)
            SROConfig sro_cfg; sro_cfg.ema_alpha = 0.30f;
            ofdm_demod_->enableSROTracking(sro_cfg);
            sfo_active_ = true;
        } else {
            ofdm_demod_->disablePhaseTracker();
            ofdm_demod_->disableSROTracking();
            sfo_active_ = false;
        }
        sfo_resampler_.reset();

        // FEC
        auto blk = LDPCBlockSize::Short;
        ldpc_enc_     = std::make_unique<LDPCEncoder>(frame_p_.fec_rate, blk);
        ldpc_dec_bp_  = std::make_unique<LDPCDecoder>(frame_p_.fec_rate, blk, 25);

        ORBGRANDConfig orbcfg;
        orbcfg.max_queries = 5000;
        orbcfg.max_weight  = 4;
        ldpc_dec_orb_ = std::make_unique<ORBGRANDDecoder>(frame_p_.fec_rate, blk, orbcfg);

        interleaver_ = std::make_unique<BitInterleaver>(ldpc_enc_->codewordBits());

        // BICM-ID orchestrator. Borrows mapper / interleaver / decoder
        // pointers — recreated whenever the modulation or FEC rate
        // changes (i.e., here in initDSP).
        bicm_mapper_  = std::make_unique<SymbolMapper>(ofdm_p_.modulation);
        BICMConfig bcfg;
        bcfg.outer_iterations = std::max<size_t>(1, ecfg_.bicm_outer_iter);
        bcfg.ldpc_inner_iter  = 25;
        bcfg.use_extrinsic    = true;
        // Inner ISoftDecoder selection: BP (default) or SOGRAND/ORBGRAND.
        // Both are members declared before bicm_decoder_ (see audio_engine.hpp),
        // so the chosen inner outlives the borrowed pointer here. Default is BP:
        // measurement showed BP-inner beats SOGRAND-inner at every tested point.
        ISoftDecoder* bicm_inner = ecfg_.bicm_inner_sogrand
                                       ? static_cast<ISoftDecoder*>(ldpc_dec_orb_.get())
                                       : static_cast<ISoftDecoder*>(ldpc_dec_bp_.get());
        bicm_decoder_ = std::make_unique<BICMDecoder>(
            bicm_mapper_.get(), interleaver_.get(),
            bicm_inner, bcfg);

        // BICM-ID drives its own BP inner loop and takes precedence over
        // the ORBGRAND decoder — they can't both run on a codeword. Note
        // it once so a user who enabled ORBGRAND isn't confused that it's
        // not actually in use. (#53)
        if (ecfg_.use_bicm_id && ecfg_.bicm_outer_iter > 1 && ecfg_.use_orbgrand
            && !bicm_orbgrand_note_shown_) {
            std::fprintf(stderr,
                "[engine] note: BICM-ID is enabled and overrides ORBGRAND "
                "for the data codeword (BICM-ID uses its own BP inner loop).\n");
            bicm_orbgrand_note_shown_ = true;
        }

        // Derived sizes (LDPC capacities)
        k_bytes_  = ldpc_enc_->infoBytes();
        n_bytes_  = ldpc_enc_->codewordBytes();
        n_cw_     = ldpc_enc_->codewordBits();
        // When the RS outer code is enabled the LDPC info block carries
        // a 16-byte parity tail, so the frame payload shrinks by that
        // amount. Also clamp blocks that would exceed RS's 255-byte
        // block-size limit — fall back to no-RS in that pathological
        // case (only happens with huge LDPC matrices we don't use).
        size_t rs_overhead = 0;
        if (modem_p_.enable_rs_outer &&
            k_bytes_ <= ReedSolomon::MAX_BLOCK &&
            k_bytes_ > ReedSolomon::PARITY_BYTES + 1) {
            rs_overhead = ReedSolomon::PARITY_BYTES;
        }
        size_t k_eff = (k_bytes_ > rs_overhead) ? k_bytes_ - rs_overhead : 0;
        frame_capacity_ = (k_eff > constants::FRAME_OVERHEAD)
                           ? k_eff - constants::FRAME_OVERHEAD : 0;

        // Opus — bitrate sized so a single packet fits in the LDPC info
        // capacity per OFDM-frame slot. Each codeword takes ~N_cw/(bps × Nd)
        // OFDM symbols of duration symbol_length / sample_rate seconds, so
        //   codeword_dur_s = (N_cw / (bps × Nd)) × (sym_len / SR)
        // Opus packet size in bytes ≈ bitrate × frame_ms / 8000.
        // We want packet_bytes ≤ frame_capacity, with ~10% margin.
        OpusConfig ocfg;
        ocfg.sample_rate = kAudioSampleRate;
        ocfg.channels    = 1;
        ocfg.frame_ms    = 20.0f;        // standard streaming frame

        size_t bps   = bitsPerSymbol(ofdm_p_.modulation);
        size_t Nd    = ofdm_mod_ ? ofdm_mod_->dataSubcarriers() : 1;
        if (bps == 0 || Nd == 0) bps = 2, Nd = 1;
        size_t bits_per_sym = bps * Nd;
        double cw_syms = static_cast<double>(n_cw_) /
                         std::max<double>(1, static_cast<double>(bits_per_sym));
        double cw_dur_s = cw_syms *
                          (static_cast<double>(ofdm_p_.symbolLength())
                           / std::max<uint32_t>(1u, ofdm_p_.sample_rate));
        // Bytes-per-codeword budget × 8 / dur(s) → bits per second of audio
        size_t budget_bytes = (frame_capacity_ > 8) ? frame_capacity_ - 8 : 1;
        double max_bps = static_cast<double>(budget_bytes) * 8.0 /
                          std::max(1e-3, cw_dur_s);
        // Apply a 10% safety margin and clamp to Opus's valid range
        uint32_t target = static_cast<uint32_t>(max_bps * 0.9);
        if (target < 6000)   target = 6000;
        if (target > 510000) target = 510000;
        // For very low capacity, raise frame_ms so the packet count drops
        if (target == 6000 && cw_dur_s > 0.025) ocfg.frame_ms = 40.0f;
        ocfg.bitrate = target;
        ocfg.vbr     = false;
        opus_enc_ = std::make_unique<OpusAudioEncoder>(ocfg);
        opus_dec_ = std::make_unique<OpusAudioDecoder>(ocfg);

        rx_sym_len_       = ofdm_p_.symbolLength();
        rx_codeword_bits_ = n_cw_;

        // PAPR reducer (tone reservation)
        papr_.reset();
        if (ecfg_.papr.enabled) {
            const auto& alloc = ofdm_mod_->allocation();
            papr_ = std::make_unique<PAPRReducer>(
                ofdm_p_.fft_size,
                ofdm_p_.guardLeft(),
                ofdm_p_.fft_size - ofdm_p_.guardRight(),
                alloc.data_indices,
                alloc.pilot_indices,
                ecfg_.papr);
            // Operate on the allocation's carved reserved tones (data-free)
            // instead of letting the reducer steal live data carriers.
            papr_->useReservedTones(alloc.reserved_indices);
        }

        // Hierarchical modulation mapper
        hier_mapper_.reset();
        if (ecfg_.hier.enabled) {
            hier_mapper_ = std::make_unique<HierarchicalMapper>(ecfg_.hier);
        }

        // ---- M/S parallel chain (Side stream over LP layer) ----
        // Activate only when hier is enabled and the bit split is symmetric
        // (hp_bps == lp_bps). With symmetric splits both codewords have the
        // same length, so the existing LDPC matrices and interleaver dimensions
        // can be reused for the Side path. Asymmetric splits fall back to the
        // single-codeword bit-split path (no true M/S separation).
        side_enc_.reset();
        side_dec_.reset();
        side_ldpc_enc_.reset();
        side_ldpc_dec_.reset();
        side_interleaver_.reset();
        ms_mode_active_ = false;
        if (hier_mapper_ && hier_mapper_->isEnabled()
            && hier_mapper_->hpBPS() == hier_mapper_->lpBPS()
            && hier_mapper_->hpBPS() > 0)
        {
            ms_mode_active_ = true;

            OpusConfig side_cfg;
            side_cfg.sample_rate = kAudioSampleRate;
            side_cfg.channels    = 1;
            // Side carries the diff signal — typically lower energy and content
            // than Mid; halve the bitrate budget to match.
            side_cfg.bitrate     = 32000;
            side_cfg.frame_ms    = 20.0f;
            side_cfg.vbr         = false;
            side_enc_ = std::make_unique<OpusAudioEncoder>(side_cfg);
            side_dec_ = std::make_unique<OpusAudioDecoder>(side_cfg);

            // Side codeword uses the SAME LDPC params as Mid for symmetric
            // bit-count alignment with the constellation.
            side_ldpc_enc_    = std::make_unique<LDPCEncoder>(frame_p_.fec_rate, blk);
            side_ldpc_dec_    = std::make_unique<LDPCDecoder>(frame_p_.fec_rate, blk, 25);
            side_interleaver_ = std::make_unique<BitInterleaver>(
                                    side_ldpc_enc_->codewordBits());

            // Allocate one Side reconstructor per stream so each stereo
            // stream maintains its own all-pass-cascade IIR state.
            // Reallocating on SR change ensures the cascade center
            // frequencies retune.
            for (auto& sr : side_recons_) {
                sr = std::make_unique<SideReconstructor>(kAudioSampleRate);
            }
        } else {
            for (auto& sr : side_recons_) sr.reset();
        }

        // ModCod detector (RX auto-configuration from PLS)
        modcod_det_.reset();

        // Wire PAPR reducer to modulator
        if (papr_ && ofdm_mod_) {
            ofdm_mod_->setPAPRReducer(papr_.get());
        }

        // Per-stream audio playback rings (decoded Opus → HW output).
        // One ring per stream so the multi-channel audio_monitor_ callback
        // can route each stream to its own output channel.
        for (auto& r : playback_rings_) {
            r = std::make_unique<RingBuffer>(PLAYBACK_RING_SIZE);
        }
        // mic_captures_ is populated lazily in syncConfig() when a stream
        // is configured with source = Microphone. Each unique input_device
        // value gets one HWAudioDevice + ring buffer.
        mic_captures_.clear();

        // Always-on audio monitor: plays decoded RX audio out the system
        // default speakers regardless of whether the modem is in loopback or
        // HW-radio mode. Without this the user hears nothing in loopback,
        // even though decoding succeeds — that was a major UX gap.
        //
        // Multi-channel mode: stream i is routed to output channel i. On a
        // 2-channel default speaker the device negotiation falls back to 2
        // channels and surplus streams (2..7) wrap modulo 2 into L/R; on a
        // multi-channel VAC the full 1:1 routing is preserved.
        audio_monitor_.reset();
        if (audio_monitor_ = std::make_unique<HWAudioDevice>(); audio_monitor_->init()) {
            HWAudioConfig pc;
            pc.sample_rate     = kAudioSampleRate;  // decoded-audio rate, not RF
            pc.buffer_frames   = 1024;
            pc.playback_device = -1;  // system default
            std::vector<RingBuffer*> rings;
            rings.reserve(PLAYBACK_RING_COUNT);
            for (auto& r : playback_rings_) rings.push_back(r.get());
            if (!audio_monitor_->startMultiChannelPlayback(rings, pc)) {
                std::fprintf(stderr,
                    "[engine] audio_monitor startMultiChannelPlayback FAILED — "
                    "decoded audio will be written to rings but NOT played out. "
                    "Check default playback device.\n");
                std::fflush(stderr);
                audio_monitor_.reset();
            }
        } else {
            std::fprintf(stderr,
                "[engine] audio_monitor init() FAILED — no playback device "
                "available.\n");
            std::fflush(stderr);
            audio_monitor_.reset();
        }

        // Mic input devices (per-stream input_device value) are started
        // lazily by syncConfig() when a stream's source = Microphone.
        // Already cleared above.

        // Reset state
        frame_number_      = 0;
        preamble_sent_     = false;
        preamble_received_ = false;
        had_sync_          = false;
        consec_bad_ticks_  = 0;
        preamble_interval_ = frame_p_.preamble_interval;
        tone_phase_        = 0.f;
        rx_accum_.clear();

        // VCM per-slot FEC tracking
        tx_vcm_last_mod_ = ofdm_p_.modulation;
        tx_vcm_last_fec_ = frame_p_.fec_rate;

        // RX PLS tracking. The PLS codeword is PLS_CODED_BITS BPSK symbols;
        // when the allocation has fewer data subcarriers than that (FFT
        // 64/128, or a narrow target_bw), TX's modulateSymbols emits
        // ceil(PLS_CODED_BITS / dataCount) OFDM symbols — RX must consume
        // the SAME count. Consuming a fixed single symbol slipped the
        // stream by the surplus every frame and nothing ever decoded at
        // those FFT sizes (the shipped "Emergency" preset among them).
        pls_received_ = false;
        rx_frame_count_ = 0;
        rx_pls_pending_ = false;
        {
            size_t dc = ofdm_mod_ ? ofdm_mod_->dataSubcarriers() : 0;
            size_t pls_syms = (dc > 0)
                ? (static_cast<size_t>(PLS_CODED_BITS) + dc - 1) / dc : 1;
            pls_samples_needed_ = pls_syms * ofdm_p_.symbolLength();
        }

        live_stats_ = ModemStats{};
        sync_fsm_.reset();
        live_stats_.sync_state = sync_fsm_.state();

    } catch (const std::exception& e) {
        emit engineError(QString("DSP init failed: %1").arg(e.what()));
    }
}

void AudioEngine::teardownDSP() {
    if (hw_audio_) {
        hw_audio_->stop();
        hw_audio_.reset();
    }
    if (audio_monitor_) {
        audio_monitor_->stop();
        audio_monitor_.reset();
    }
    // Tear down all per-device mic captures.
    for (auto& kv : mic_captures_) {
        if (kv.second.dev) kv.second.dev->stop();
    }
    mic_captures_.clear();
    modem_.reset();
    ofdm_mod_.reset();
    ofdm_demod_.reset();
    sync_.reset();
    ldpc_enc_.reset();
    ldpc_dec_bp_.reset();
    ldpc_dec_orb_.reset();
    interleaver_.reset();
    opus_enc_.reset();
    opus_dec_.reset();
    papr_.reset();
    hier_mapper_.reset();
    side_enc_.reset();
    side_dec_.reset();
    side_ldpc_enc_.reset();
    side_ldpc_dec_.reset();
    side_interleaver_.reset();
    for (auto& sr : side_recons_) sr.reset();
    ms_mode_active_ = false;
    for (auto& r : playback_rings_) r.reset();
}

// =========================================================================
// Configuration sync (read AppState under lock)
// =========================================================================

void AudioEngine::syncConfig() {
    std::array<StreamConfig, MAX_STREAMS> snapshot_streams;
    bool hier_changed = false;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        ofdm_p_     = state_.ofdm;
        frame_p_    = state_.frame;
        modem_p_    = state_.modem;
        tx_enabled_ = state_.tx_enabled;
        tx_gain_db_ = state_.tx_gain_db;
        snapshot_streams = state_.stream_configs;
        // Propagate the configured signal bandwidth into OFDMParams so
        // the auto-guard calculation keeps active subcarriers inside the
        // configured LPF window. Without this, OFDM-occupied BW can
        // exceed signal_bw and the TX LPF (cutoff = signal_bw × 1.1)
        // clips the edge data subcarriers — silent data loss.
        ofdm_p_.target_bw_hz = state_.modem.signal_bw;
        // Sync hierarchical mod config from AppState. The engine's
        // ecfg_.hier is the authoritative working copy used by initDSP;
        // AppState's `hier` is the user-facing edit point exposed to
        // panels (TX, LinkBudget, dialogs).
        if (ecfg_.hier.mode      != state_.hier.mode      ||
            ecfg_.hier.alpha     != state_.hier.alpha     ||
            ecfg_.hier.enabled   != state_.hier.enabled   ||
            ecfg_.hier.hp_bits   != state_.hier.hp_bits   ||
            ecfg_.hier.lp_bits   != state_.hier.lp_bits)
        {
            ecfg_.hier   = state_.hier;
            hier_changed = true;
        }
    }
    (void)hier_changed;  // currently informational; reconfigure path
                         // in onConfigChanged already triggers initDSP
    // Push stream configs into the coordinator (creates / tears down codecs).
    for (size_t i = 0; i < MAX_STREAMS; ++i) {
        streams_.configureStream(i, snapshot_streams[i]);
    }

    // Open / close per-device mic captures based on enabled streams.
    // Group enabled mic streams by their input_device value. For each
    // unique device id, ensure a capture device + ring exists. Close
    // any devices no longer referenced by an enabled mic stream.
    std::set<int> needed_devices;
    for (const auto& sc : snapshot_streams) {
        if (sc.enabled && sc.source == StreamAudioSource::Microphone) {
            needed_devices.insert(sc.input_device);
        }
    }
    // Open newly-needed devices.
    for (int dev_id : needed_devices) {
        if (mic_captures_.count(dev_id)) continue;
        MicCapture mc;
        mc.ring = std::make_unique<RingBuffer>(PLAYBACK_RING_SIZE);
        mc.dev  = std::make_unique<HWAudioDevice>();
        if (mc.dev->init()) {
            HWAudioConfig cap_cfg;
            cap_cfg.sample_rate    = kAudioSampleRate;  // audio rate, not RF
            cap_cfg.capture_device = dev_id;
            if (mc.dev->startCaptureOnly(*mc.ring, cap_cfg)) {
                mic_captures_.emplace(dev_id, std::move(mc));
            } else {
                std::fprintf(stderr,
                    "[engine] mic capture failed to start on device %d\n",
                    dev_id);
                std::fflush(stderr);
            }
        }
    }
    // Close devices that nothing references anymore.
    for (auto it = mic_captures_.begin(); it != mic_captures_.end(); ) {
        if (needed_devices.count(it->first) == 0) {
            if (it->second.dev) it->second.dev->stop();
            it = mic_captures_.erase(it);
        } else {
            ++it;
        }
    }

    // Load / unload file-source WAVs per stream. Lazy: only re-read when the
    // stream's path changes or transitions in/out of File source mode.
    for (size_t i = 0; i < MAX_STREAMS; ++i) {
        const auto& sc = snapshot_streams[i];
        bool want_file = sc.enabled && sc.source == StreamAudioSource::File &&
                         sc.file_path[0] != '\0';
        if (!want_file) {
            file_sources_[i].reset();
            continue;
        }
        std::string p(sc.file_path);
        if (file_sources_[i] && file_sources_[i]->path == p) continue;

        // Read the WAV. Reuse the cli/wav_io reader.
        auto src = std::make_unique<FileSource>();
        src->path = p;
        src->sample_rate = sc.sample_rate;

        // Tiny inline WAV reader (PCM 16-bit or float32, any channel count
        // — collapsed to mono by averaging). Avoids pulling in cli/wav_io.hpp.
        std::FILE* f = std::fopen(p.c_str(), "rb");
        if (!f) {
            file_sources_[i].reset();
            continue;
        }
        std::fseek(f, 0, SEEK_END);
        long fsize = std::ftell(f);
        std::fseek(f, 0, SEEK_SET);
        if (fsize < 44) { std::fclose(f); file_sources_[i].reset(); continue; }
        std::vector<uint8_t> buf(static_cast<size_t>(fsize));
        std::fread(buf.data(), 1, buf.size(), f);
        std::fclose(f);
        if (std::memcmp(buf.data(), "RIFF", 4) != 0 ||
            std::memcmp(buf.data() + 8, "WAVE", 4) != 0) {
            file_sources_[i].reset();
            continue;
        }
        // Find fmt + data chunks
        size_t pos = 12;
        uint16_t fmt_code = 0, channels = 0, bits = 0;
        uint32_t sr_in = 0, data_len = 0;
        size_t data_off = 0;
        while (pos + 8 <= buf.size()) {
            uint32_t cklen = static_cast<uint32_t>(buf[pos+4]) |
                             (static_cast<uint32_t>(buf[pos+5]) << 8) |
                             (static_cast<uint32_t>(buf[pos+6]) << 16) |
                             (static_cast<uint32_t>(buf[pos+7]) << 24);
            if (std::memcmp(&buf[pos], "fmt ", 4) == 0) {
                fmt_code = static_cast<uint16_t>(buf[pos+8] | (buf[pos+9] << 8));
                channels = static_cast<uint16_t>(buf[pos+10] | (buf[pos+11] << 8));
                sr_in    = static_cast<uint32_t>(buf[pos+12]) |
                           (static_cast<uint32_t>(buf[pos+13]) << 8) |
                           (static_cast<uint32_t>(buf[pos+14]) << 16) |
                           (static_cast<uint32_t>(buf[pos+15]) << 24);
                bits     = static_cast<uint16_t>(buf[pos+22] | (buf[pos+23] << 8));
            } else if (std::memcmp(&buf[pos], "data", 4) == 0) {
                data_off = pos + 8;
                data_len = cklen;
                break;
            }
            pos += 8 + cklen;
        }
        if (data_off == 0 || channels == 0) {
            file_sources_[i].reset();
            continue;
        }
        size_t bytes_per = bits / 8;
        size_t total_samples = data_len / bytes_per;
        size_t frames = total_samples / channels;
        src->samples.resize(frames);
        if (fmt_code == 1 && bits == 16) {
            const int16_t* s = reinterpret_cast<const int16_t*>(buf.data() + data_off);
            for (size_t fi = 0; fi < frames; ++fi) {
                int32_t mix = 0;
                for (size_t c = 0; c < channels; ++c) mix += s[fi*channels + c];
                src->samples[fi] = static_cast<float>(mix) /
                                    (32768.f * static_cast<float>(channels));
            }
        } else if (fmt_code == 3 && bits == 32) {
            const float* s = reinterpret_cast<const float*>(buf.data() + data_off);
            for (size_t fi = 0; fi < frames; ++fi) {
                float mix = 0.f;
                for (size_t c = 0; c < channels; ++c) mix += s[fi*channels + c];
                src->samples[fi] = mix / static_cast<float>(channels);
            }
        } else {
            file_sources_[i].reset();
            continue;
        }
        // Resample to the stream rate via polyphase L/M rational
        // resampler. Replaces the previous linear-interpolation pass
        // which audibly distorted high-frequency content. The polyphase
        // path runs once at file-load time so the cost is amortized
        // across the entire loop playback duration — no per-tick
        // overhead.
        if (sr_in != sc.sample_rate && sr_in > 0) {
            auto gcd = [](uint32_t a, uint32_t b) {
                while (b) { uint32_t t = b; b = a % b; a = t; }
                return a;
            };
            uint32_t g = gcd(sc.sample_rate, sr_in);
            size_t L = sc.sample_rate / g;
            size_t M = sr_in / g;
            // Clamp ratios that would blow up memory (e.g., a 7777 Hz
            // input to 48000 Hz output → L/M = 48000/7777 ≈ 6.17 but
            // L = 48000, M = 7777 → interp_mid_size = src.size() * 48000.
            // For pathological co-prime sample rates fall back to the
            // previous linear path.
            if (L * src->samples.size() < 64 * 1024 * 1024) {
                RationalResampler rs(L, M);
                std::vector<float> resampled(rs.outputCount(src->samples.size())
                                              + static_cast<size_t>(L) + 16);
                size_t n_out = rs.process(src->samples.data(),
                                           src->samples.size(),
                                           resampled.data());
                resampled.resize(n_out);
                src->samples.swap(resampled);
            } else {
                // Pathological ratio — fall back to linear to avoid
                // multi-gigabyte intermediate buffers.
                double ratio = static_cast<double>(sc.sample_rate) / sr_in;
                size_t new_n = static_cast<size_t>(
                    src->samples.size() * ratio);
                std::vector<float> resampled(new_n);
                for (size_t k = 0; k < new_n; ++k) {
                    double src_idx = k / ratio;
                    size_t i0 = static_cast<size_t>(src_idx);
                    if (i0 >= src->samples.size() - 1) {
                        resampled[k] = src->samples.back();
                        continue;
                    }
                    float frac = static_cast<float>(src_idx - i0);
                    resampled[k] = src->samples[i0] * (1.f - frac) +
                                   src->samples[i0 + 1] * frac;
                }
                src->samples.swap(resampled);
            }
        }
        src->pos = 0;
        file_sources_[i] = std::move(src);
    }
}

void AudioEngine::onConfigChanged() {
    // Called from GUI thread via signal — will be processed in engine thread
    // Full DSP re-init on config change (modcod change requires new encoder/decoder)
    if (!running_.load()) return;  // Not running — nothing to reinit
    // COALESCE queued rebuilds: each one is expensive (~1+ s with HW audio:
    // device close + reopen), and a burst of GUI changes (slider drag,
    // preset hopping) queues one PER change. Only the LAST queued rebuild
    // reflects the final config — the stale ones just serialize seconds of
    // dead time in front of it (and in front of a pending Stop: the GUI
    // walker measured 6+ s of frozen UI from exactly this). Each queued
    // lambda skips itself unless it is the newest, or a shutdown began.
    const uint64_t gen = ++config_gen_;
    QMetaObject::invokeMethod(this, [this, gen]() {
        if (gen != config_gen_.load()) return;       // superseded
        if (shutdown_requested_.load()) return;      // stopping anyway
        if (timer_) timer_->stop();
        teardownDSP();
        initDSP();
        if (timer_) timer_->start(static_cast<int>(ecfg_.process_interval_ms));
    }, Qt::QueuedConnection);
}

void AudioEngine::setEngineConfig(const AudioEngineConfig& cfg) {
    // Must be thread-safe: GUI thread writes, engine thread reads ecfg_. The
    // write is serialized with the engine thread's reads by running on that
    // thread (queued); ecfg_mtx_ additionally guards against the GUI thread's
    // engineConfig() copy reading ecfg_ mid-write.
    if (running_.load()) {
        AudioEngineConfig copy = cfg;
        QMetaObject::invokeMethod(this, [this, copy]() {
            std::lock_guard<std::mutex> lk(ecfg_mtx_);
            ecfg_ = copy;
        }, Qt::QueuedConnection);
    } else {
        // Engine not running — no engine-thread reader, but lock anyway in
        // case the GUI calls engineConfig() concurrently.
        std::lock_guard<std::mutex> lk(ecfg_mtx_);
        ecfg_ = cfg;
    }
}

// =========================================================================
// VCM per-slot FEC rebuild
// =========================================================================

void AudioEngine::rebuildFEC(FECRate rate, Modulation mod) {
    // A confirmed PLS/VCM/AMC modulation change must rebuild the OFDM
    // mod/demod too: the SymbolMapper is baked into both, and the RX
    // codeword slicing derives from bitsPerOFDMSymbol(). The old signature
    // ignored the modulation, so a remote TX's constellation switch kept
    // being demapped at the stale order forever. The fresh demodulator
    // starts with a cold channel estimate — drop the sync latch so the
    // next periodic preamble re-seeds it via the wide-scan re-acquire,
    // exactly like first acquisition.
    if (mod != ofdm_p_.modulation) {
        ofdm_p_.modulation = mod;
        ofdm_mod_   = std::make_unique<OFDMModulator>(ofdm_p_);
        ofdm_demod_ = std::make_unique<OFDMDemodulator>(ofdm_p_);
        if (ecfg_.use_mmse) ofdm_demod_->enableMMSE();
        if (modem_p_.enable_afc) {
            ofdm_demod_->enablePhaseTracker();
            SROConfig sro_cfg; sro_cfg.ema_alpha = 0.30f;
            ofdm_demod_->enableSROTracking(sro_cfg);
        }
        preamble_received_ = false;
        rx_pls_pending_    = false;
    }

    auto blk = LDPCBlockSize::Short;
    ldpc_enc_     = std::make_unique<LDPCEncoder>(rate, blk);
    ldpc_dec_bp_  = std::make_unique<LDPCDecoder>(rate, blk, 25);

    ORBGRANDConfig orbcfg;
    orbcfg.max_queries = 5000;
    orbcfg.max_weight  = 4;
    ldpc_dec_orb_ = std::make_unique<ORBGRANDDecoder>(rate, blk, orbcfg);

    interleaver_ = std::make_unique<BitInterleaver>(ldpc_enc_->codewordBits());

    // BICM-ID orchestrator follows the new mapper / interleaver / decoder.
    bicm_mapper_  = std::make_unique<SymbolMapper>(ofdm_p_.modulation);
    BICMConfig bcfg;
    bcfg.outer_iterations = std::max<size_t>(1, ecfg_.bicm_outer_iter);
    bcfg.ldpc_inner_iter  = 25;
    bcfg.use_extrinsic    = true;
    // Inner ISoftDecoder selection (BP default, SOGRAND opt-in) — mirrors initDSP.
    ISoftDecoder* bicm_inner = ecfg_.bicm_inner_sogrand
                                   ? static_cast<ISoftDecoder*>(ldpc_dec_orb_.get())
                                   : static_cast<ISoftDecoder*>(ldpc_dec_bp_.get());
    bicm_decoder_ = std::make_unique<BICMDecoder>(
        bicm_mapper_.get(), interleaver_.get(),
        bicm_inner, bcfg);

    k_bytes_  = ldpc_enc_->infoBytes();
    n_bytes_  = ldpc_enc_->codewordBytes();
    n_cw_     = ldpc_enc_->codewordBits();
    rx_codeword_bits_ = n_cw_;

    // BUG FIX (audit): the previous version didn't account for the RS
    // outer-code parity tail, so frame_capacity_ was 16 bytes too LARGE
    // when RS was enabled. The frame builder would size payload from
    // the bloated capacity, then the RS encoder couldn't fit it into
    // k_bytes_. Mirror the initDSP logic exactly.
    size_t rs_overhead = 0;
    if (modem_p_.enable_rs_outer &&
        k_bytes_ <= ReedSolomon::MAX_BLOCK &&
        k_bytes_ > ReedSolomon::PARITY_BYTES + 1) {
        rs_overhead = ReedSolomon::PARITY_BYTES;
    }
    size_t k_eff = (k_bytes_ > rs_overhead) ? k_bytes_ - rs_overhead : 0;
    frame_capacity_ = (k_eff > constants::FRAME_OVERHEAD)
                       ? k_eff - constants::FRAME_OVERHEAD : 0;
}

// =========================================================================
// RX PLS detection (decode PLS from post-preamble OFDM symbol)
// =========================================================================

// Consume and decode the PLS block from the front of rx_accum_. TX prepends
// it to EVERY data codeword (per-frame ModCod signaling); it spans
// ceil(PLS_CODED_BITS / dataCount) OFDM symbols (one at FFT >= 256 with
// typical allocations, 2-3 at FFT 64/128). The RX must strip exactly that
// many — a fixed one-symbol strip slips the stream by the surplus every
// frame and every codeword fails CRC. The caller guarantees the samples
// are present.
void AudioEngine::processRXPLS() {
    if (!ofdm_demod_ || rx_accum_.size() < pls_samples_needed_) return;

    // Demodulate each PLS OFDM symbol → concatenated equalized subcarriers.
    const size_t sym_len = ofdm_p_.symbolLength();
    ComplexBuf pls_data_syms;
    for (size_t off = 0; off + sym_len <= pls_samples_needed_; off += sym_len) {
        ComplexBuf pls_sym(
            rx_accum_.begin() + static_cast<ptrdiff_t>(off),
            rx_accum_.begin() + static_cast<ptrdiff_t>(off + sym_len));
        ComplexBuf eq;
        ofdm_demod_->demodulate(pls_sym, eq);
        pls_data_syms.insert(pls_data_syms.end(), eq.begin(), eq.end());
    }
    rx_accum_.erase(rx_accum_.begin(),
                    rx_accum_.begin() + static_cast<ptrdiff_t>(pls_samples_needed_));

    // BPSK soft values (real part; > 0 ⇒ bit 0). decodePLSSoft soft-combines
    // the repeated Reed-Muller copies that fit in the data subcarriers and
    // FHT-decodes them — coding + combining gain over the old hard scheme (#26).
    std::vector<float> pls_soft;
    pls_soft.reserve(pls_data_syms.size());
    for (const auto& s : pls_data_syms) pls_soft.push_back(s.real());

    PLSBlock pls;
    bool decoded = (pls_soft.size() >= PLS_CODED_BITS &&
                    decodePLSSoft(pls_soft, pls));
    if (decoded) {
        pls_received_ = true;
        // Feed the ModCod detector; a confirmed change rebuilds the RX FEC.
        bool changed = modcod_det_.feed(pls);
        if (changed) {
            rebuildFEC(modcod_det_.currentFECRate(),
                       modcod_det_.currentModulation());
            modcod_det_.acknowledge();
        }
        // Push PLS state to GUI diagnostic widget with the real running
        // confirmation count from the detector.
        bridge_.pushPLS(static_cast<int>(pls.modulation),
                        static_cast<int>(pls.fec_rate),
                        pls.vcm_slot, pls.vcm_total,
                        /*crc_ok=*/true,
                        modcod_det_.confirmationCount());
    }
}

// =========================================================================
// HW-audio auto-recovery — runs once every RECOVERY_TICK_PERIOD ticks
// =========================================================================

void AudioEngine::recoverAudioDevicesIfNeeded() {
    if (++diag_recovery_counter_ < RECOVERY_TICK_PERIOD) return;
    diag_recovery_counter_ = 0;

    // audio_monitor_: always-on multi-channel playback of decoded RX audio.
    // If a USB device was unplugged or the OS audio service restarted, the
    // device transitions to !isRunning() without us tearing it down. Try
    // to rebuild it onto the current default device with the same rings.
    if (audio_monitor_ && !audio_monitor_->isRunning()) {
        audio_monitor_->stop();  // ensure inner state is cleared
        HWAudioConfig pc;
        pc.sample_rate     = kAudioSampleRate;  // decoded-audio rate, not RF
        pc.buffer_frames   = 1024;
        pc.playback_device = -1;
        std::vector<RingBuffer*> rings;
        rings.reserve(PLAYBACK_RING_COUNT);
        for (auto& r : playback_rings_) rings.push_back(r.get());
        if (!audio_monitor_->startMultiChannelPlayback(rings, pc)) {
            // Failed — leave the device object alive so the next check
            // can retry. Don't reset() it because that would lose the
            // device-list cache and force re-enumeration.
            std::fprintf(stderr, "[engine] audio_monitor restart failed\n");
            std::fflush(stderr);
        } else {
            std::fprintf(stderr, "[engine] audio_monitor recovered\n");
            std::fflush(stderr);
        }
    }

    // Mic captures (per-device): same recovery logic applied to each
    // open device. If any specific device dropped (USB unplug, driver
    // restart) we try to re-acquire just that one.
    for (auto& kv : mic_captures_) {
        auto& mc = kv.second;
        if (mc.dev && !mc.dev->isRunning() && mc.ring) {
            mc.dev->stop();
            HWAudioConfig cap_cfg;
            cap_cfg.sample_rate    = kAudioSampleRate;  // audio rate, not RF
            cap_cfg.capture_device = kv.first;
            if (!mc.dev->startCaptureOnly(*mc.ring, cap_cfg)) {
                std::fprintf(stderr,
                    "[engine] mic capture restart failed on device %d\n",
                    kv.first);
                std::fflush(stderr);
            } else {
                std::fprintf(stderr,
                    "[engine] mic capture recovered on device %d\n",
                    kv.first);
                std::fflush(stderr);
            }
        }
    }
}

// =========================================================================
// Main processing tick
// =========================================================================

void AudioEngine::processTick() {
    auto tick_start = std::chrono::steady_clock::now();
    if (!modem_ || !ofdm_mod_) return;

    // 1. Read current TX state from AppState
    float rx_gain_db = 0.f;
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        tx_enabled_ = state_.tx_enabled;
        tx_gain_db_ = state_.tx_gain_db;
        rx_gain_db  = state_.modem.rx_gain_db;
    }
    // Apply RX gain live (no DSP rebuild) so the Tuning slider is smooth.
    if (modem_) modem_->setRxGainDb(rx_gain_db);

    // Per-stage timing. When a tick exceeds 25 ms we log the breakdown so
    // we can see whether TX, RX, or something else is the heavy stage.
    auto now = []{ return std::chrono::steady_clock::now(); };
    auto t0 = now();

    // 2. TX path
    if (tx_enabled_) {
        processTX();
    } else if (prev_tx_enabled_ && modem_) {
        // TX just transitioned on→off: ramp the carrier down smoothly so
        // the un-key doesn't radiate broadband splatter (issue #5).
        modem_->endTransmission();
    }
    prev_tx_enabled_ = tx_enabled_;
    auto t_tx = now();

    // 3. Loopback processing (connect TX→RX)
    if (modem_->isLoopback()) {
        if (ecfg_.loopback_snr >= 0.f) {
            modem_->processLoopbackAWGN(ecfg_.loopback_snr,
                                         frame_number_ + 12345);
        }
    }
    auto t_lb = now();

    // 4. RX path
    processRX();
    auto t_rx = now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    double tx_ms = ms(t0, t_tx);
    double lb_ms = ms(t_tx, t_lb);
    double rx_ms = ms(t_lb, t_rx);
    if (tx_ms + lb_ms + rx_ms > 25.0) {
        std::fprintf(stderr,
            "[engine] slow tick: TX=%.1fms LB=%.1fms RX=%.1fms total=%.1fms\n",
            tx_ms, lb_ms, rx_ms, tx_ms + lb_ms + rx_ms);
        std::fflush(stderr);
    }

    // Bump the diagnostic-feed throttle counter at the end of every tick.
    ++diag_tick_counter_;

    // Auto-recover HW audio devices that the OS may have dropped (USB
    // hot-unplug, sleep/wake, driver restart). Internally rate-limited
    // to once every ~3 s so the check itself is free in the common case.
    recoverAudioDevicesIfNeeded();

    // Record this tick's wall-clock time for latency monitoring.
    float tick_ms = static_cast<float>(
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - tick_start).count());
    tick_latencies_[tick_latency_pos_] = tick_ms;
    tick_latency_pos_ = (tick_latency_pos_ + 1) % TICK_LATENCY_WINDOW;
    float t_max = 0.f, t_sum = 0.f;
    for (float v : tick_latencies_) {
        t_sum += v;
        if (v > t_max) t_max = v;
    }
    live_stats_.tick_max_ms = t_max;
    live_stats_.tick_avg_ms = t_sum / static_cast<float>(TICK_LATENCY_WINDOW);

    // 5. Push measurements to AppState
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.stats = live_stats_;
        state_.agc_gain_db = modem_ ? modem_->agcGainDB() : 0.f;
    }
}

// =========================================================================
// TX Path
// =========================================================================

void AudioEngine::generateTestAudio(float* pcm, size_t samples) {
    float freq = ecfg_.tone_freq_hz;
    float amp  = ecfg_.tone_amplitude;
    float sr   = static_cast<float>(kAudioSampleRate);  // audio rate, not RF
    float phase_inc = 2.f * static_cast<float>(M_PI) * freq / sr;

    // Apply TX gain
    float gain_lin = std::pow(10.f, tx_gain_db_ / 20.f);
    amp *= gain_lin;

    for (size_t i = 0; i < samples; ++i) {
        pcm[i] = amp * std::sin(tone_phase_);
        tone_phase_ += phase_inc;
    }

    // Keep phase wrapped to avoid float precision loss
    if (tone_phase_ > 2.f * static_cast<float>(M_PI) * 1000.f) {
        tone_phase_ = std::fmod(tone_phase_,
                                 2.f * static_cast<float>(M_PI));
    }
}

void AudioEngine::processTX() {
    // Defensive: bail if any DSP component is missing (config change in flight).
    // Log the first few skips so the user can diagnose "TX does nothing" —
    // these silent early-exits used to make TX failures completely invisible.
    if (!opus_enc_ || !ldpc_enc_ || !ofdm_mod_ || !modem_ || !interleaver_) {
        static int reported = 0;
        if (reported++ < 3) {
            std::fprintf(stderr,
                "[engine] processTX skipped (DSP not ready): "
                "opus=%d ldpc=%d ofdm=%d modem=%d inter=%d\n",
                opus_enc_ ? 1 : 0, ldpc_enc_ ? 1 : 0, ofdm_mod_ ? 1 : 0,
                modem_ ? 1 : 0, interleaver_ ? 1 : 0);
            std::fflush(stderr);
        }
        return;
    }
    if (frame_capacity_ == 0) {
        static int reported = 0;
        if (reported++ < 3) {
            std::fprintf(stderr,
                "[engine] processTX skipped: frame_capacity=0 "
                "(k_bytes=%zu, RS=%d). Try disabling RS or using a "
                "stronger FEC rate.\n",
                k_bytes_, modem_p_.enable_rs_outer ? 1 : 0);
            std::fflush(stderr);
        }
        return;
    }

    // VCM: determine current slot's ModCod (if VCM enabled)
    Modulation tx_mod = ofdm_p_.modulation;
    FECRate    tx_fec = frame_p_.fec_rate;
    if (ecfg_.vcm.enabled && ecfg_.vcm.num_slots > 1) {
        const auto& slot = ecfg_.vcm.slotForFrame(frame_number_);
        tx_mod = slot.modulation;
        tx_fec = slot.fec_rate;

        // Per-slot FEC re-initialization: rebuild encoder/decoder when
        // the VCM slot changes ModCod from the previous frame
        if (tx_mod != tx_vcm_last_mod_ || tx_fec != tx_vcm_last_fec_) {
            rebuildFEC(tx_fec, tx_mod);
            tx_vcm_last_mod_ = tx_mod;
            tx_vcm_last_fec_ = tx_fec;
        }
    }

    // Send preamble at start and periodically
    if (!preamble_sent_ ||
        (preamble_interval_ > 0 && frame_number_ % preamble_interval_ == 0)) {
        ComplexBuf preamble = ofdm_mod_->generatePreamble();
        modem_->transmit(preamble);

        // Push preamble samples to spectrum analyzer
        std::vector<float> pre_real(preamble.size());
        for (size_t i = 0; i < preamble.size(); ++i) {
            pre_real[i] = preamble[i].real();
        }
        bridge_.pushSpectrumSamples(pre_real.data(), pre_real.size());

        preamble_sent_ = true;
    }

    // Transmit PLS block (always BPSK, sent as known bytes → OFDM modulated)
    {
        PLSBlock pls;
        if (ecfg_.vcm.enabled) {
            pls = ecfg_.vcm.plsForFrame(frame_number_);
        } else {
            pls.modulation = tx_mod;
            pls.fec_rate   = tx_fec;
            pls.vcm_active = false;
            pls.vcm_slot   = 0;
            pls.vcm_total  = 1;
        }
        // Reed-Muller coded PLS, mapped to GENUINE BPSK (independent of the
        // data modcod) and repeated to fill the symbol's data subcarriers so
        // the RX can soft-combine the copies (#26). One OFDM symbol.
        std::vector<uint8_t> coded;
        encodePLSCoded(pls, coded);            // PLS_CODED_BITS = 112
        size_t data_count = computeAllocation(ofdm_p_).dataCount();
        size_t n_copies = (data_count >= coded.size())
                          ? data_count / coded.size() : 1;
        ComplexBuf pls_syms;
        pls_syms.reserve(n_copies * coded.size());
        for (size_t c = 0; c < n_copies; ++c)
            for (uint8_t b : coded)
                pls_syms.emplace_back(b ? -1.f : 1.f, 0.f);  // BPSK: 0→+1, 1→−1
        ComplexBuf pls_bb;
        ofdm_mod_->modulateSymbols(pls_syms.data(), pls_syms.size(), pls_bb);
        modem_->transmit(pls_bb);
    }

    // Generate audio frame for the legacy single-stream path. Each enabled
    // stream's audio source synthesizes (or captures) one Opus-frame worth
    // of samples. Per-stream encoders feed the FrameBuilder directly so the
    // 8-stream setup actually carries 8 distinct audio channels on-air.
    size_t audio_samples = opus_enc_->config().frameSamples();

    // Snapshot stream configs once so we don't lock per stream.
    std::array<StreamConfig, MAX_STREAMS> snap_streams{};
    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        snap_streams = state_.stream_configs;
    }

    // Build the FrameBuilder. If any stream is enabled, emit per-stream
    // packets; otherwise fall back to a single-packet test-tone path.
    FrameBuilder fb(frame_capacity_);

    // Synthesize one Opus frame worth of audio per enabled stream and push
    // to the coordinator's input ring (Microphone source bypasses synthesis
    // since the capture device already feeds into that ring). Then encode
    // through the per-stream encoder via streams_.encodeIntoFrame(fb).
    // When M/S mode is active, cache stream 0 and stream 1 synthesized PCM
    // so the M/S TX branch below can feed stream 1 audio into the Side
    // encoder (LP layer of the hierarchical constellation). This replaces
    // the previous synthetic 1.5× side tone with real audio from a
    // configured stream — the graceful-degradation behavior now applies
    // to a real audio source.
    std::vector<float> ms_stream1_pcm_cache;

    bool any_stream_emitted = false;
    for (uint8_t id = 0; id < MAX_STREAMS; ++id) {
        const auto& sc = snap_streams[id];
        if (!sc.enabled) continue;
        size_t samples_total = audio_samples * sc.channels;

        if (sc.source == StreamAudioSource::Microphone) {
            // Drain THIS stream's assigned mic capture ring into its
            // encoder input. Per-device mapping: streams with the same
            // input_device share one ring (they get IDENTICAL captured
            // audio); streams with different input_device values pull
            // from independent rings, so two physical mics can drive
            // two different streams simultaneously.
            auto it = mic_captures_.find(sc.input_device);
            if (it != mic_captures_.end() && it->second.ring) {
                std::vector<float> mic_pcm(samples_total, 0.f);
                size_t got = it->second.ring->read(mic_pcm.data(),
                                                    samples_total);
                if (got > 0) {
                    streams_.pushTX(id, mic_pcm.data(), got);
                }
            }
            any_stream_emitted = true;
            continue;
        }

        std::vector<float> pcm(samples_total, 0.f);
        if (sc.source == StreamAudioSource::File && file_sources_[id]) {
            auto& fs = *file_sources_[id];
            // Loop the WAV. Mono → broadcast to all channels.
            for (size_t i = 0; i < audio_samples; ++i) {
                if (fs.samples.empty()) break;
                float v = fs.samples[fs.pos];
                fs.pos = (fs.pos + 1) % fs.samples.size();
                if (sc.channels == 2) {
                    pcm[2*i + 0] = v;
                    pcm[2*i + 1] = v;
                } else {
                    pcm[i] = v;
                }
            }
        } else if (sc.source == StreamAudioSource::TestTone) {
            float fs = static_cast<float>(sc.sample_rate);
            float dphi = 2.f * static_cast<float>(M_PI) * sc.tone_freq_hz / fs;
            for (size_t i = 0; i < audio_samples; ++i) {
                float v = sc.tone_amplitude * std::sin(stream_tone_phase_[id]);
                stream_tone_phase_[id] += dphi;
                if (stream_tone_phase_[id] > 2.f * static_cast<float>(M_PI))
                    stream_tone_phase_[id] -= 2.f * static_cast<float>(M_PI);
                if (sc.channels == 2) {
                    pcm[2*i + 0] = v;
                    pcm[2*i + 1] = v;
                } else {
                    pcm[i] = v;
                }
            }
        }
        // Silence + File (not implemented) leave pcm as zeros.
        streams_.pushTX(id, pcm.data(), pcm.size());
        // Cache stream 1 PCM (mono) for the M/S branch. We pull mono
        // regardless of the stream's channel config because side_enc_ is
        // built for mono input.
        if (id == 1 && ms_mode_active_) {
            ms_stream1_pcm_cache.assign(audio_samples, 0.f);
            if (sc.channels == 2) {
                for (size_t i = 0; i < audio_samples; ++i) {
                    ms_stream1_pcm_cache[i] = 0.5f *
                        (pcm[2*i + 0] + pcm[2*i + 1]);
                }
            } else {
                std::memcpy(ms_stream1_pcm_cache.data(), pcm.data(),
                            audio_samples * sizeof(float));
            }
        }
        any_stream_emitted = true;
    }

    // ---- Build Mid AND Side frames in one pass (per-stream M/S) ----
    // When hier mod is symmetric-M/S active, each stereo stream's
    // (L+R)/2 → Mid packets and (L−R)/2 → Side packets. Mono streams
    // contribute Mid only. Side frame stays empty when hier is OFF or
    // no stereo streams are enabled.
    bool hier_ms_active = ms_mode_active_ && hier_mapper_ &&
                          hier_mapper_->isEnabled() && side_ldpc_enc_ &&
                          side_interleaver_;
    size_t side_capacity = 0;
    if (hier_ms_active) {
        size_t side_k = side_ldpc_enc_->infoBytes();
        bool side_rs = (modem_p_.enable_rs_outer &&
                        side_k > ReedSolomon::PARITY_BYTES + 1 &&
                        side_k <= ReedSolomon::MAX_BLOCK);
        size_t side_k_eff = side_rs ? side_k - ReedSolomon::PARITY_BYTES
                                     : side_k;
        side_capacity = (side_k_eff > constants::FRAME_OVERHEAD)
                          ? side_k_eff - constants::FRAME_OVERHEAD : 0;
    }
    FrameBuilder side_fb(side_capacity);

    if (any_stream_emitted) {
        streams_.encodeIntoFrames(fb, hier_ms_active ? &side_fb : nullptr,
                                   hier_ms_active);
    } else {
        // Legacy: tone via the engine's single Opus encoder.
        std::vector<float> pcm(audio_samples, 0.f);
        if (ecfg_.generate_tone) {
            generateTestAudio(pcm.data(), audio_samples);
        }
        std::vector<uint8_t> opus_pkt;
        if (!opus_enc_->encode(pcm.data(), opus_pkt)) return;
        if (!fb.addPacket(0, opus_pkt.data(), opus_pkt.size())) return;
    }

    auto frame_data = fb.build(frame_number_, tx_fec, tx_mod);
    // Wrap with Reed-Solomon outer code when enabled: data occupies
    // k_bytes_ - 16, last 16 bytes are RS parity. On RX, the parity
    // bytes are stripped after correction. Falls back to plain LDPC
    // when the block size is outside RS's 255-byte limit.
    bool rs_active = (modem_p_.enable_rs_outer &&
                      k_bytes_ <= ReedSolomon::MAX_BLOCK &&
                      k_bytes_ > ReedSolomon::PARITY_BYTES + 1);
    size_t rs_data_len = rs_active ? (k_bytes_ - ReedSolomon::PARITY_BYTES)
                                    : k_bytes_;
    frame_data.resize(rs_data_len, 0);
    if (rs_active) {
        // Extend buffer to make room for RS parity, then encode in-place.
        frame_data.resize(k_bytes_, 0);
        rs_.encode(frame_data.data(), rs_data_len);
    }
    frame_data.resize(k_bytes_, 0);

    // LDPC encode
    std::vector<uint8_t> codeword(n_bytes_, 0);
    ldpc_enc_->encode(frame_data.data(), codeword.data());

    // Interleave
    std::vector<uint8_t> interleaved(n_bytes_, 0);
    interleaver_->interleave(codeword.data(), interleaved.data());

    // OFDM modulate. Three branches:
    //   1. M/S hierarchical (stereo, symmetric HP/LP split):
    //      Two parallel codewords — Mid on HP layer, Side on LP layer.
    //   2. Bit-split hierarchical (asymmetric or test-tone path):
    //      One codeword, first portion → HP, remainder → LP.
    //   3. Plain (no hierarchy): straight bit-mapping.
    ComplexBuf tx_bb;
    if (hier_ms_active) {
        // ---- Build the SIDE codeword from the side FrameBuilder ----
        // side_fb was already populated above by streams_.encodeIntoFrames
        // with per-stream Side packets (one per enabled stereo stream).
        // Each packet is the Opus encoding of that stream's
        // (L−R)/2 signal. If no stereo streams are enabled, side_fb is
        // empty and the LP layer carries CRC + zero-payload — the
        // listener gets silence on Side, predictive recovery kicks in
        // if HP-only stays locked.
        size_t side_k_bytes = side_ldpc_enc_->infoBytes();
        auto side_frame = side_fb.build(frame_number_, tx_fec, tx_mod);
        bool side_rs_active = (modem_p_.enable_rs_outer &&
                                side_k_bytes <= ReedSolomon::MAX_BLOCK &&
                                side_k_bytes > ReedSolomon::PARITY_BYTES + 1);
        size_t side_rs_data_len = side_rs_active
            ? (side_k_bytes - ReedSolomon::PARITY_BYTES)
            : side_k_bytes;
        side_frame.resize(side_rs_data_len, 0);
        if (side_rs_active) {
            side_frame.resize(side_k_bytes, 0);
            rs_.encode(side_frame.data(), side_rs_data_len);
        }
        side_frame.resize(side_k_bytes, 0);

        std::vector<uint8_t> side_codeword(side_ldpc_enc_->codewordBytes(), 0);
        side_ldpc_enc_->encode(side_frame.data(), side_codeword.data());

        std::vector<uint8_t> side_interleaved(side_codeword.size(), 0);
        side_interleaver_->interleave(side_codeword.data(),
                                       side_interleaved.data());

        // ---- Hierarchical map: Mid (multi-stream Mid frame) → HP,
        //      Side (multi-stream Side frame) → LP ----
        uint8_t hp_bps = hier_mapper_->hpBPS();
        uint8_t lp_bps = hier_mapper_->lpBPS();
        size_t mid_bits  = n_cw_;
        size_t side_bits = side_ldpc_enc_->codewordBits();
        size_t num_syms = std::min(mid_bits / hp_bps, side_bits / lp_bps);

        ComplexBuf hier_syms;
        hier_mapper_->map(interleaved.data(),     num_syms * hp_bps,
                          side_interleaved.data(), num_syms * lp_bps,
                          hier_syms);
        ofdm_mod_->modulateSymbols(hier_syms.data(), hier_syms.size(), tx_bb);

    } else if (hier_mapper_ && hier_mapper_->isEnabled()) {
        // Asymmetric hierarchical: single codeword, bit-split into HP/LP.
        uint8_t hp_bps = hier_mapper_->hpBPS();
        uint8_t lp_bps = hier_mapper_->lpBPS();
        uint8_t total_bps = hp_bps + lp_bps;
        size_t total_bits = n_cw_;
        size_t num_syms = total_bits / total_bps;
        size_t hp_total_bits = num_syms * hp_bps;
        size_t lp_total_bits = num_syms * lp_bps;

        // The codeword is split into a contiguous HP partition (bits
        // [0, hp_total_bits)) followed by a contiguous LP partition
        // (bits [hp_total_bits, hp_total_bits+lp_total_bits)). The RX
        // asymmetric branch reconstructs exactly this layout by
        // concatenating HP-then-LP LLRs with NO byte padding. The
        // previous TX read LP from a BYTE-aligned offset
        // ((hp_total_bits+7)/8), which skips (8 - hp_total_bits%8) bits
        // whenever hp_total_bits isn't a multiple of 8 — guaranteeing an
        // LP decode failure on every such preset (e.g. QPSK/QAM256,
        // hp_total_bits=540, offset 4 bits). Repack the LP partition to a
        // byte-aligned buffer starting at the exact bit hp_total_bits so
        // map() (which reads its lp_bits pointer from bit 0) consumes the
        // same bits RX expects. HP starts at bit 0, so it needs no repack.
        auto getBit = [](const uint8_t* d, size_t i) -> uint8_t {
            return (d[i >> 3] >> (7 - (i & 7))) & 1;
        };
        std::vector<uint8_t> lp_packed((lp_total_bits + 7) / 8, 0);
        for (size_t j = 0; j < lp_total_bits; ++j) {
            if (getBit(interleaved.data(), hp_total_bits + j)) {
                lp_packed[j >> 3] |=
                    static_cast<uint8_t>(1u << (7 - (j & 7)));
            }
        }

        ComplexBuf hier_syms;
        hier_mapper_->map(interleaved.data(), hp_total_bits,
                          lp_packed.data(),    lp_total_bits,
                          hier_syms);
        ofdm_mod_->modulateSymbols(hier_syms.data(), hier_syms.size(), tx_bb);
    } else {
        ofdm_mod_->modulateBits(interleaved.data(), n_cw_, tx_bb);
    }

    // Transmit
    modem_->transmit(tx_bb);

    // Push TX baseband to spectrum analyzer (every tick — analyzer has its
    // own decimation/lock, doesn't cross-thread copy a vector).
    // Throttle the spectrum push to ~33 Hz (every 3rd tick at 10 ms
    // ticks). Each push deep-copies tx_real (1000+ floats at FFT≥1024)
    // across the engine/GUI thread boundary via Qt::QueuedConnection,
    // and the bridge runs an FFT on each push. At 100 Hz this floods
    // the GUI event queue and shows up as TX-induced hiccups; 33 Hz
    // is visually indistinguishable from 100 Hz on the waterfall.
    if ((diag_tick_counter_ % 3) == 0) {
        std::vector<float> tx_real(tx_bb.size());
        for (size_t i = 0; i < tx_bb.size(); ++i) {
            tx_real[i] = tx_bb[i].real();
        }
        bridge_.pushSpectrumSamples(tx_real.data(), tx_real.size());
        // Scope: throttle to 1 in 5 ticks (~20 Hz). Cross-thread vector copy.
        if (tx_real.size() >= 8 && (diag_tick_counter_ % 15) == 0) {
            std::vector<float> scope_samples;
            scope_samples.reserve(tx_real.size() / 8);
            for (size_t i = 0; i < tx_real.size(); i += 8) scope_samples.push_back(tx_real[i]);
            bridge_.pushScopeSamples(scope_samples.data(), scope_samples.size());
        }
    }

    // Update TX level meter (RMS) + PAPR + clip indicator
    float tx_power = 0.f;
    float tx_peak  = 0.f;
    for (auto& s : tx_bb) {
        float p = std::norm(s);
        if (p > tx_peak) tx_peak = p;
        tx_power += p;
    }
    tx_power /= static_cast<float>(tx_bb.size());
    float tx_rms_db = (tx_power > 1e-20f)
        ? 10.f * std::log10(tx_power) : -60.f;
    float tx_peak_dbfs = (tx_peak > 1e-20f)
        ? 10.f * std::log10(tx_peak) : -60.f;
    float papr_db = (tx_power > 1e-20f)
        ? 10.f * std::log10(tx_peak / tx_power) : 0.f;

    // Stamp this frame's TX time so we can compute round-trip latency on RX.
    auto tx_now = std::chrono::steady_clock::now();
    pending_tx_times_[frame_number_ % PENDING_RING] = tx_now;

    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.tx_meter.update(tx_rms_db, 0.01f);
    }

    live_stats_.papr_tx_db   = papr_db;
    live_stats_.tx_peak_dbfs = tx_peak_dbfs;
    live_stats_.tx_clipping  = (tx_peak_dbfs >= -0.1f);

    live_stats_.frames_tx++;
    frame_number_++;
}

// =========================================================================
// RX Path
// =========================================================================

void AudioEngine::updateRxMeasurements() {
    if (!ofdm_demod_) return;

    // Running post-FEC frame-error rate.
    if (live_stats_.frames_rx > 0) {
        live_stats_.ber_estimate =
            static_cast<float>(live_stats_.frames_bad) /
            static_cast<float>(live_stats_.frames_rx);
    }

    // SNR / EVM from the demodulator's channel estimate.
    live_stats_.snr_db = ofdm_demod_->snrEstimate();
    float nv = ofdm_demod_->noiseVariance();
    if (nv > 0.f && nv < 100.f) {
        live_stats_.evm_percent = 100.f * std::sqrt(nv);
    }

    // Combined CFO: integer (bin shift) + fractional (tracked phase).
    live_stats_.integer_cfo_bins = ofdm_demod_->lastIntegerCFO();
    if (ofdm_p_.fft_size > 0) {
        float sc_spacing = static_cast<float>(ofdm_p_.sample_rate) /
                            ofdm_p_.fft_size;
        live_stats_.cfo_total_hz =
            static_cast<float>(live_stats_.integer_cfo_bins) * sc_spacing
            + ofdm_demod_->trackedPhaseRad() * sc_spacing /
              (2.f * static_cast<float>(M_PI));
    }
    live_stats_.clock_ppm = ofdm_demod_->clockPpm();

    // Close the SFO loop: drive the RX resampler from the pilot-slope
    // estimate (PI timing-recovery loop). Called once per codeword from
    // each RX branch, matching the per-codeword cadence validated in
    // sfo_test.cpp. The new ratio takes effect on the next tick's resample.
    if (sfo_active_) {
        sfo_resampler_.nudge(live_stats_.clock_ppm, SFO_KP, SFO_KI);
    }
}

void AudioEngine::processRX() {
    // Defensive: a config change races with the running timer. Bail until all
    // RX-path components have been re-instantiated by initDSP().
    if (!ofdm_demod_ || !modem_ || !ofdm_mod_ || !interleaver_ ||
        !ldpc_dec_bp_ || !opus_dec_) return;

    // Receive available baseband samples
    ComplexBuf rx_new = modem_->receive(ofdm_p_.sample_rate / 10);
    if (rx_new.empty()) return;

    // Update RX level meter
    float rx_power = 0.f;
    for (auto& s : rx_new) rx_power += std::norm(s);
    rx_power /= static_cast<float>(rx_new.size());
    float rx_rms_db = (rx_power > 1e-20f)
        ? 10.f * std::log10(rx_power) : -60.f;

    {
        std::lock_guard<std::mutex> lock(state_.mtx);
        state_.rx_meter.update(rx_rms_db, 0.01f);
        state_.agc_gain_db = modem_->agcGainDB();
    }

    // AGC pumping detector: peak-to-peak gain swing over the last AGC_WINDOW
    // RX calls. Healthy AGC ripple is < 2 dB; > 6 dB usually indicates a
    // feedback loop or chasing burst noise. Tracked under live_stats_ so the
    // GUI can light an indicator and (optionally) trigger an alarm.
    {
        float g = modem_->agcGainDB();
        agc_history_[agc_history_pos_] = g;
        agc_history_pos_ = (agc_history_pos_ + 1) % AGC_WINDOW;
        if (agc_history_filled_ < AGC_WINDOW) ++agc_history_filled_;
        // Scan only the entries actually written. Scanning the full
        // zero-initialized window for the first AGC_WINDOW (64) RX calls
        // made gmin≈0 → a fictitious 20-40 dB ripple that lit the AGC-pump
        // alarm red on every startup / reconfigure. (#51)
        float gmin = agc_history_[0], gmax = agc_history_[0];
        for (size_t i = 1; i < agc_history_filled_; ++i) {
            if (agc_history_[i] < gmin) gmin = agc_history_[i];
            if (agc_history_[i] > gmax) gmax = agc_history_[i];
        }
        live_stats_.agc_ripple_db = gmax - gmin;
    }

    // Record post-downconvert IQ to file if active. Done BEFORE accumulation
    // so the recorded stream is exactly what the demod saw.
    if (iq_recorder_.isOpen() && !rx_new.empty()) {
        iq_recorder_.write(rx_new.data(), rx_new.size());
    }

    // If IQ playback is active, OVERRIDE rx_new with samples read from the
    // playback file (rather than what the modem actually receives). This lets
    // the user replay a captured file as if it were live RX input. Guard the
    // player access against a concurrent startIQPlayback() on the GUI thread.
    if (iq_player_active_.load()) {
        std::lock_guard<std::mutex> lk(io_mtx_);
        if (iq_player_active_.load() && !iq_player_.atEnd()) {
            size_t want = ofdm_p_.sample_rate / 10;
            ComplexBuf playback(want);
            size_t got = iq_player_.read(playback.data(), want);
            playback.resize(got);
            rx_new.swap(playback);
        }
    }

    // Accumulate for symbol-aligned processing. When AFC is on, run the
    // stream through the SFO resampler first so the symbol grid stays
    // aligned despite the TX/RX soundcard clock offset (the resampler ratio
    // is closed-loop driven by the SRO estimate below). At step≈1 the
    // resampler is an exact, ~1-sample-delayed copy, so a near-zero offset
    // costs nothing.
    if (sfo_active_) {
        sfo_resampler_.process(rx_new.data(), rx_new.size(), rx_accum_);
    } else {
        rx_accum_.insert(rx_accum_.end(), rx_new.begin(), rx_new.end());
    }

    // Hard cap: a producer that sustainedly outpaces the per-tick codeword
    // drain (MAX_CODEWORDS_PER_TICK) — e.g. fast IQ-file replay or a stalled
    // decode — would otherwise grow rx_accum_ without bound (OOM + rising
    // O(n) front-erase cost). Keep at most ~2 s of samples; drop the oldest.
    const size_t RX_ACCUM_CAP =
        static_cast<size_t>(ofdm_p_.sample_rate ? ofdm_p_.sample_rate : 48000u) * 2;
    if (rx_accum_.size() > RX_ACCUM_CAP) {
        rx_accum_.erase(rx_accum_.begin(),
                        rx_accum_.begin() +
                        static_cast<ptrdiff_t>(rx_accum_.size() - RX_ACCUM_CAP));
    }

    // Feed the eye-diagram widget — throttled. The engine ticks at ~100 Hz;
    // pushing 19 KB vectors across the Qt event queue at that rate burns
    // through CPU just deep-copying for queued signals and starves the GUI
    // (the multi-second hang/brief-update pattern). At 1 in 5 ticks (~20 Hz)
    // the eye is still visually smooth and the GUI keeps up.
    if (rx_new.size() > 0 && ofdm_p_.fft_size > 0 &&
        (diag_tick_counter_ % 5) == 0) {
        std::vector<float> eye_samples;
        eye_samples.reserve(rx_new.size());
        for (auto& s : rx_new) eye_samples.push_back(s.real());
        bridge_.pushEyeSamples(eye_samples, static_cast<int>(ofdm_p_.fft_size));
    }

    // Process preamble if we haven't yet (look for enough samples)
    if (!preamble_received_) {
        size_t short_total = 10 * (ofdm_p_.fft_size / 4);
        size_t long_total  = 2 * ofdm_p_.symbolLength();
        size_t preamble_len = short_total + long_total;

        // Re-acquisition is handled below by a wide fineSync ACQUIRE scan (the
        // had_sync_ branch): after sync loss the preamble is no longer front-
        // aligned and may sit an arbitrary number of samples into rx_accum_, so
        // we cross-correlate against the known ZC long-preamble body to find its
        // true start. (This replaces the old Schmidl-Cox coarse trim, whose
        // [0,N) CP-autocorrelation search could only localize within a symbol
        // and peaked at every boundary — so it could realign to the WRONG one.)
        if (rx_accum_.size() >= preamble_len) {
            // Fine-timing acquisition. The Schmidl-Cox CP autocorrelation only
            // snaps to *a* symbol boundary — its metric peaks at EVERY symbol
            // edge, so on re-acquire it can land a whole symbol early/late, and
            // even on a front-aligned burst a few samples of group delay leave
            // the FFT window misplaced. fineSync cross-correlates the incoming
            // stream against the known long-preamble body (the Zadoff-Chu CAZAC
            // symbol, whose sharp autocorrelation makes this peak unambiguous)
            // to pin the long symbol's start to the sample. It searches a ±N/8
            // window around the body guess; we centre that guess on the nominal
            // body position (short preamble + first CP). We then back out where
            // the whole preamble begins so the slice below and the consume-erase
            // stay consistent.
            //
            // Common case (first-ever, front-aligned burst) stays byte-stable:
            // the search is centred on the nominal body offset with the narrow
            // ±N/8 window, the ZC autocorrelation peaks there, and pre_start
            // resolves to 0.
            //
            // Re-acquisition (had_sync_) is the hard case: a HW-audio dropout can
            // leave the preamble an arbitrary number of samples into rx_accum_,
            // more than one symbol — beyond what coarseSync's [0,N) CP search can
            // localize. There we hand fineSync a wide search_range so it SCANS
            // the buffer for the ZC body and locks onto the true start, rather
            // than trusting coarseSync to have landed on the right boundary.
            size_t pre_start = 0;
            const size_t cp = ofdm_p_.cpLength();
            if (sync_) {
                SyncResult fr;
                bool found = false;
                if (had_sync_) {
                    // Wide ACQUIRE scan over the whole buffer.
                    int centre = static_cast<int>(rx_accum_.size() / 2);
                    found = sync_->fineSync(rx_accum_, centre, fr,
                                            rx_accum_.size());
                } else {
                    // Narrow refine around the nominal front-aligned body
                    // (keeps the common loopback case byte-stable), then a
                    // wide-scan fallback: a first acquire over real audio
                    // (HW capture started independently of the TX, plus
                    // FIR group delays) puts the preamble at an arbitrary
                    // offset the +/-N/8 window can't reach.
                    int body_guess = static_cast<int>(short_total + cp);
                    found = sync_->fineSync(rx_accum_, body_guess, fr);
                    if (!(found && fr.valid)) {
                        int centre = static_cast<int>(rx_accum_.size() / 2);
                        found = sync_->fineSync(rx_accum_, centre, fr,
                                                rx_accum_.size());
                    }
                }
                if (found && fr.valid) {
                    // fr.timing_offset is the long-symbol BODY start; the full
                    // preamble begins cp+short_total samples earlier. Clamp to
                    // a valid, fully-buffered window so the erase/slice below
                    // can never run off either end.
                    long ps = static_cast<long>(fr.timing_offset)
                            - static_cast<long>(cp)
                            - static_cast<long>(short_total);
                    if (ps < 0) ps = 0;
                    if (static_cast<size_t>(ps) + preamble_len > rx_accum_.size())
                        ps = static_cast<long>(rx_accum_.size() - preamble_len);
                    pre_start = static_cast<size_t>(ps);
                } else {
                    // VALIDITY GATE (audit): no ZC body anywhere in the
                    // buffer — there is no preamble here, only noise, idle
                    // hiss, or data tails. The old code fell through with
                    // pre_start = 0 and processPreamble() "succeeded" on
                    // whatever was buffered (its only check is length),
                    // latching a fake lock that an RX keyed up before its
                    // TX could never escape — recovery was gated on
                    // had_sync_, which requires a CRC-good frame that never
                    // comes. Keep one preamble-length of tail (a real
                    // preamble can straddle the chunk boundary), report the
                    // miss to the FSM, and scan again next tick.
                    live_stats_.sync_state = sync_fsm_.feed(false, 0.0f, 1.0f);
                    if (rx_accum_.size() > preamble_len) {
                        rx_accum_.erase(
                            rx_accum_.begin(),
                            rx_accum_.begin() + static_cast<ptrdiff_t>(
                                rx_accum_.size() - preamble_len));
                    }
                    return;
                }
            }

            // Extract long training symbols for channel estimation
            ComplexBuf long_syms(
                rx_accum_.begin() + static_cast<ptrdiff_t>(pre_start + short_total),
                rx_accum_.begin() + static_cast<ptrdiff_t>(pre_start + preamble_len));

            if (ofdm_demod_->processPreamble(long_syms)) {
                preamble_received_ = true;
                // This preamble marks frame 0 of the (re)acquired stream; the
                // per-codeword loop below counts from here to skip the next
                // periodic preamble at the right cadence.
                rx_frame_count_ = 0;
                rx_pls_pending_ = false;
                live_stats_.snr_db = ofdm_demod_->snrEstimate();
                // Feed the FSM: preamble detected, perfect correlation,
                // BER unknown yet (assume good for the acquire path).
                live_stats_.sync_state = sync_fsm_.feed(true, 1.0f, 0.0f);

                // Push channel-response trace to the diagnostic widget.
                // We compute |H(k)| in dB across the active subcarriers only.
                const auto& alloc = ofdm_demod_->allocation();
                const auto& H     = ofdm_demod_->channelEstimate();
                std::vector<float> mag_db;
                mag_db.reserve(alloc.dataCount() + alloc.pilotCount());
                auto pushDb = [&](size_t k) {
                    if (k < H.size()) {
                        float m = std::abs(H[k]);
                        mag_db.push_back(m > 1e-12f
                            ? 20.f * std::log10(m) : -80.f);
                    }
                };
                // Iterate the ACTIVE bins in ascending-frequency order. The
                // allocation is DC-centered in natural FFT order (+f in
                // [1, N/2), -f above N/2), so the old contiguous
                // [guardLeft, N-guardRight) walk no longer matches the
                // active set — it crossed the Nyquist guard hole and gave
                // the widget a garbled frequency axis.
                std::vector<bool> act(ofdm_p_.fft_size, false);
                for (size_t k : alloc.data_indices)  act[k] = true;
                for (size_t k : alloc.pilot_indices) act[k] = true;
                const size_t Nfft = ofdm_p_.fft_size;
                for (size_t k = Nfft / 2; k < Nfft; ++k)   // -Nyquist .. -df
                    if (act[k]) pushDb(k);
                for (size_t k = 0; k < Nfft / 2; ++k)      // DC .. +Nyquist
                    if (act[k]) pushDb(k);
                bridge_.pushChannelResponse(mag_db);
            } else {
                // No valid preamble decode this attempt
                live_stats_.sync_state = sync_fsm_.feed(false, 0.0f, 1.0f);
            }

            // Consume everything up to AND including the preamble. With fine
            // timing, the real preamble may start pre_start samples into the
            // buffer (re-acquire landed early, or there was leading slack); drop
            // that prefix too so the payload that follows is front-aligned for
            // the per-codeword loop.
            rx_accum_.erase(
                rx_accum_.begin(),
                rx_accum_.begin() +
                    static_cast<ptrdiff_t>(pre_start + preamble_len));
        }
        return; // Wait for preamble before decoding data
    }

    // The sync FSM is driven AFTER the decode loop below, from the actual
    // per-codeword CRC outcomes (the old code fed it an unconditional "good
    // frame" here every tick, so it never reported loss and RX never
    // re-acquired). Snapshot the frame counters so we can tell what happened
    // this tick.
    const auto ok_before  = live_stats_.frames_ok;
    const auto bad_before = live_stats_.frames_bad;

    if (rx_sym_len_ == 0) return;

    // Preamble geometry — TX re-inserts a full preamble every preamble_interval_
    // frames, so the per-frame loop skips one at that cadence.
    const size_t short_total  = 10 * (ofdm_p_.fft_size / 4);
    const size_t preamble_len = short_total + 2 * ofdm_p_.symbolLength();

    // Each on-wire frame is [preamble (every preamble_interval-th)] + [PLS
    // symbol] + [data codeword]. Consume them as a unit per iteration so RX
    // stays byte-locked to the TX framing. (The previous code consumed the
    // preamble + PLS exactly once, then drained bare codewords — slipping by
    // one PLS symbol every frame, so every CRC after the first failed: the
    // "loopback never decodes" bug.) Cap codewords/tick to bound per-tick
    // wall-clock; surplus stays in rx_accum_ for the next tick.
    constexpr size_t MAX_CODEWORDS_PER_TICK = 2;
    size_t codewords_this_tick = 0;
    while (codewords_this_tick < MAX_CODEWORDS_PER_TICK) {
        size_t bits_per_sym = ofdm_mod_->bitsPerOFDMSymbol();
        if (bits_per_sym == 0) break;
        size_t syms_per_cw    = (n_cw_ + bits_per_sym - 1) / bits_per_sym;
        size_t samples_per_cw = syms_per_cw * rx_sym_len_;

        // rx_pls_pending_: the previous iteration consumed this frame's
        // preamble/PLS but the (post-PLS-rebuild) codeword wasn't fully
        // buffered yet — resume HERE without re-stripping anything.
        if (!rx_pls_pending_) {
            // frame 0's preamble was consumed by the acquisition block above,
            // hence the rx_frame_count_ > 0 guard.
            bool skip_preamble = (preamble_interval_ > 0 && rx_frame_count_ > 0 &&
                                  rx_frame_count_ % preamble_interval_ == 0);
            size_t need = (skip_preamble ? preamble_len : 0)
                        + pls_samples_needed_ + samples_per_cw;
            if (rx_accum_.size() < need) break;

            if (skip_preamble) {
                // Refresh the channel estimate from the retransmitted
                // preamble, then consume it.
                ComplexBuf long_syms(
                    rx_accum_.begin() + static_cast<ptrdiff_t>(short_total),
                    rx_accum_.begin() + static_cast<ptrdiff_t>(preamble_len));
                ofdm_demod_->processPreamble(long_syms);
                rx_accum_.erase(rx_accum_.begin(),
                                rx_accum_.begin() + static_cast<ptrdiff_t>(preamble_len));
            }

            // Strip + decode this frame's PLS block (per-frame ModCod
            // signalling).
            processRXPLS();
            ++rx_frame_count_;
            rx_pls_pending_ = true;

            // The PLS may have just switched the modcod (VCM per-slot, AMC,
            // or a remote TX change): rebuildFEC inside processRXPLS updates
            // n_cw_, so the codeword geometry computed at loop top is stale
            // for exactly the frame that FOLLOWS the announcing PLS — it was
            // sliced with the old length and the stream shifted by the
            // difference, CRC-failing every frame until resync. Recompute
            // with the post-PLS parameters before slicing.
            bits_per_sym = ofdm_mod_->bitsPerOFDMSymbol();
            if (bits_per_sym == 0) break;
            syms_per_cw    = (n_cw_ + bits_per_sym - 1) / bits_per_sym;
            samples_per_cw = syms_per_cw * rx_sym_len_;
        }

        // If the (possibly re-sized) codeword isn't fully buffered, leave it
        // for the next tick; rx_pls_pending_ remembers we already consumed
        // this frame's preamble + PLS.
        if (rx_accum_.size() < samples_per_cw) break;
        rx_pls_pending_ = false;
        ++codewords_this_tick;

        // Symbol-timing tracking. Over a long burst the RX soundcard clock
        // drifts against the TX clock, so the FFT window slowly slides off the
        // CP. trackTiming() runs the CP early/late correlation on the next
        // symbol and returns a -1/0/+1 nudge; we apply it to the window by
        // dropping one extra leading sample (+1, window late) or holding one
        // back (-1, window early, achieved by duplicating the front sample so
        // the slice starts one sample earlier). The accumulator gain inside
        // trackTiming is small (0.05), so on a drift-free stream it returns 0
        // every time and this is a no-op — which keeps the loopback tests (no
        // clock offset) byte-stable. Guarded so a nudge can never under/overrun
        // rx_accum_.
        if (timing_track_enabled_ && sync_ &&
            rx_accum_.size() >= samples_per_cw + 2) {
            int nudge = sync_->trackTiming(rx_accum_);
            if (nudge > 0) {
                // Window is late → advance by discarding one leading sample.
                rx_accum_.erase(rx_accum_.begin());
            } else if (nudge < 0) {
                // Window is early → retard by repeating the leading sample so
                // the codeword slice begins one sample sooner.
                rx_accum_.insert(rx_accum_.begin(), rx_accum_.front());
            }
        }

        // Extract one codeword's worth of OFDM symbols (the data following PLS).
        ComplexBuf cw_samples(
            rx_accum_.begin(),
            rx_accum_.begin() + static_cast<ptrdiff_t>(samples_per_cw));
        rx_accum_.erase(rx_accum_.begin(),
                        rx_accum_.begin() + static_cast<ptrdiff_t>(samples_per_cw));

        float noise_var = ofdm_demod_->noiseVariance();
        if (noise_var < 1e-10f) noise_var = 0.01f;

        // ---- M/S branch: parallel HP/LP demap from hierarchical constellation ----
        if (ms_mode_active_ && hier_mapper_ && hier_mapper_->isEnabled() &&
            side_interleaver_ && side_ldpc_dec_ && side_dec_)
        {
            // Aggregate equalized data symbols across the codeword span
            ComplexBuf agg_syms;
            for (size_t base = 0; base + rx_sym_len_ <= cw_samples.size();
                 base += rx_sym_len_) {
                ComplexBuf one_sym(
                    cw_samples.begin() + static_cast<ptrdiff_t>(base),
                    cw_samples.begin() + static_cast<ptrdiff_t>(base + rx_sym_len_));
                ComplexBuf data_syms;
                ofdm_demod_->demodulate(one_sym, data_syms);
                agg_syms.insert(agg_syms.end(), data_syms.begin(), data_syms.end());
            }
            // Batched constellation push: ONE lock acquisition after
            // accumulating all OFDM-symbol points instead of locking
            // per symbol. With ~10 OFDM symbols per codeword the old
            // code took the GUI's `state_.mtx` 10 times per RX tick,
            // starving the constellation widget's paint events and
            // showing up as TX-induced GUI hiccups.
            if (!agg_syms.empty()) {
                std::lock_guard<std::mutex> lock(state_.mtx);
                state_.constellation.push(agg_syms);
            }
            // Soft-demap HP and LP from the same constellation symbols
            std::vector<float> mid_llrs, side_llrs;
            hier_mapper_->demapSoftHP(agg_syms, noise_var, mid_llrs);
            hier_mapper_->demapSoftLP(agg_syms, noise_var, side_llrs);

            mid_llrs.resize(n_cw_, 0.f);
            side_llrs.resize(side_ldpc_dec_->codewordBits(), 0.f);

            // De-interleave each independently
            std::vector<float> mid_deint(n_cw_);
            interleaver_->deinterleave(mid_llrs.data(), mid_deint.data());
            std::vector<float> side_deint(side_ldpc_dec_->codewordBits());
            side_interleaver_->deinterleave(side_llrs.data(), side_deint.data());

            // LDPC decode each — capture posterior confidence for the
            // hierarchical layer status indicator.
            // side_info MUST be sized to the Side code's INFO bytes
            // (ceil(k/8)), exactly like mid_info uses k_bytes_. The decoder
            // writes only ceil(k/8) bytes (ldpc.hpp); the previous
            // codewordBits()/8+1 over-sized it to ceil(n/8), which (a) made
            // the RS-active guard (k <= 255) false so Side RS unwrap was
            // silently skipped, and (b) put the CRC at the wrong offset so the
            // Side/LP layer never CRC-parsed — stereo always fell back to a
            // synthesized Side. Use the same info-byte count the TX built with.
            const size_t side_k_bytes = (side_ldpc_dec_->infoBits() + 7) / 8;
            std::vector<uint8_t> mid_info(k_bytes_, 0);
            std::vector<uint8_t> side_info(side_k_bytes, 0);
            auto mid_res  = ldpc_dec_bp_->decode(mid_deint.data(), mid_info.data());
            auto side_res = side_ldpc_dec_->decode(side_deint.data(), side_info.data());

            live_stats_.frames_rx++;

            // Apply Reed-Solomon outer-code correction to BOTH Mid and
            // Side info blocks before frame parsing — fix for a real
            // bug where the M/S RX path skipped RS unwrap (the TX path
            // wraps both codewords with RS, so the RX must unwrap or
            // see corrupted CRC).
            auto apply_rs_to = [this](std::vector<uint8_t>& info_block,
                                       size_t k_bytes) {
                bool rs_active = (modem_p_.enable_rs_outer &&
                                   k_bytes <= ReedSolomon::MAX_BLOCK &&
                                   k_bytes > ReedSolomon::PARITY_BYTES + 1);
                if (!rs_active) return;
                size_t rs_data_len = k_bytes - ReedSolomon::PARITY_BYTES;
                std::vector<uint8_t> rs_block(k_bytes, 0);
                std::memcpy(rs_block.data(), info_block.data(),
                            std::min(info_block.size(), k_bytes));
                if (rs_.decode(rs_block.data(), rs_data_len) >= 0) {
                    info_block.assign(
                        rs_block.begin(),
                        rs_block.begin() + static_cast<ptrdiff_t>(rs_data_len));
                }
            };
            apply_rs_to(mid_info, k_bytes_);
            apply_rs_to(side_info, side_k_bytes);

            // Parse each frame separately
            ParsedFrame mid_pf, side_pf;
            FrameParser::parse(mid_info, mid_pf);
            FrameParser::parse(side_info, side_pf);

            // Dispatch BOTH frames to the multi-stream coordinator.
            // For each stream_id present in mid_pf, the coordinator
            // decodes Mid via rx_mid_[id] and, if the matching Side
            // packet is in side_pf, also decodes Side via rx_side_[id].
            // Output is written to per-stream rx_output_[id] rings
            // (currently Mid-only PCM — full L/R stereo output is a
            // future iteration with stereo playback rings).
            streams_.onParsedFrames(mid_pf, &side_pf);

            // No legacy mid_pcm/side_pcm reconstruction here anymore —
            // the per-stream Mid/Side decode happens inside
            // streams_.onParsedFrames.
            std::vector<float> mid_pcm, side_pcm;
            (void)mid_pcm; (void)side_pcm;

            // ---- Hierarchical layer stats (graceful-degradation view) ----
            live_stats_.hier_active     = true;
            if (mid_pf.crc_ok) ++live_stats_.hp_frames_ok;
            else               ++live_stats_.hp_frames_bad;
            if (side_pf.crc_ok) ++live_stats_.lp_frames_ok;
            else                ++live_stats_.lp_frames_bad;
            live_stats_.hp_avg_llr_mag = mid_res.avg_magnitude;
            live_stats_.lp_avg_llr_mag = side_res.avg_magnitude;
            live_stats_.hp_locked = mid_pf.crc_ok;
            live_stats_.lp_locked = side_pf.crc_ok;
            if (mid_pf.crc_ok && side_pf.crc_ok) {
                live_stats_.frames_ok++;
            } else {
                // Was never counted — the M/S branch only tracked
                // hp/lp_frames_bad, so the top-level BER readout stayed
                // stuck at 0 in stereo hierarchical mode. (#9)
                live_stats_.frames_bad++;
            }

            // Route M/S decoded audio with predictive Side reconstruction.
            // Three cases driven by HP/LP decoder state + LDPC posterior
            // confidence (see SideReconstructor::confidenceFromLLR):
            //   (a) HP locked AND LP locked with high |LLR| → transmitted
            //       Side plays through unmodified.
            //   (b) HP locked AND LP locked with marginal |LLR| → cross-
            //       fade between transmitted and synthesized Side based
            //       on the LLR-derived confidence. Smooth handoff, no
            //       click at the transition.
            //   (c) HP locked AND LP failed → synthesized Side from Mid
            //       via the all-pass-cascade decorrelator. Stereo image
            //       persists (decorrelated copy of Mid) instead of
            //       collapsing to identical-on-both-channels mono.
            //   (d) HP failed → no output. RX squelch / sync-loss alarms
            //       cover this case.
            // Per-stream Mid/Side decoding + predictive Side recovery
            // now happens inside streams_.onParsedFrames (above).
            // Each enabled stream's Mid PCM goes into its rx_output_
            // ring; Side PCM is captured in latestSide() for recorders
            // and a future stereo-output stage. The per-stream playback
            // ring drain below (`streams_.activeCount() > 0` block) is
            // unchanged — it pulls Mid from each stream's ring and
            // routes to its assigned output channel.
            //
            // Future iteration: when playback rings expand to support
            // stereo per stream, the side_recon_ + LLR-confidence
            // crossfade will move into the per-stream drain stage so
            // each stereo stream gets its own graceful-degradation
            // crossfade independently.
            updateRxMeasurements();   // refresh SNR/EVM/CFO/PPM/BER (#9)
            continue;  // skip the single-codeword path below
        }

        // ---- Asymmetric hierarchical RX branch ----
        // When hierarchical mod is enabled but HP and LP bit counts are
        // asymmetric (e.g., QPSK_QAM64 = 2+4), the TX uses the bit-split
        // single-codeword mapping at audio_engine.cpp:~1028. We must
        // demap with the hierarchical mapper here, not the uniform
        // SymbolMapper that the plain single-codeword path uses below.
        // The single codeword is split into HP and LP partitions; both
        // need to be soft-demapped and concatenated to reconstruct the
        // original interleaved bit stream.
        if (hier_mapper_ && hier_mapper_->isEnabled() && !ms_mode_active_) {
            ComplexBuf agg_syms;
            for (size_t base = 0; base + rx_sym_len_ <= cw_samples.size();
                 base += rx_sym_len_) {
                ComplexBuf one_sym(
                    cw_samples.begin() + static_cast<ptrdiff_t>(base),
                    cw_samples.begin() + static_cast<ptrdiff_t>(base + rx_sym_len_));
                ComplexBuf data_syms;
                ofdm_demod_->demodulate(one_sym, data_syms);
                agg_syms.insert(agg_syms.end(), data_syms.begin(), data_syms.end());
            }
            // Batched constellation push — one lock per codeword.
            if (!agg_syms.empty()) {
                std::lock_guard<std::mutex> lock(state_.mtx);
                state_.constellation.push(agg_syms);
            }
            // Demap HP and LP layers from the same symbols, then
            // concatenate to match the TX bit-split layout.
            std::vector<float> hp_llrs, lp_llrs;
            hier_mapper_->demapSoftHP(agg_syms, noise_var, hp_llrs);
            hier_mapper_->demapSoftLP(agg_syms, noise_var, lp_llrs);
            std::vector<float> all_llrs;
            all_llrs.reserve(hp_llrs.size() + lp_llrs.size());
            all_llrs.insert(all_llrs.end(), hp_llrs.begin(), hp_llrs.end());
            all_llrs.insert(all_llrs.end(), lp_llrs.begin(), lp_llrs.end());
            all_llrs.resize(n_cw_, 0.f);

            std::vector<float> deintlv(n_cw_);
            interleaver_->deinterleave(all_llrs.data(), deintlv.data());

            std::vector<uint8_t> decoded(k_bytes_, 0);
            auto dec_res = ldpc_dec_bp_->decode(deintlv.data(), decoded.data());
            live_stats_.frames_rx++;

            ParsedFrame parsed;
            if (FrameParser::parse(decoded, parsed) && parsed.crc_ok) {
                live_stats_.frames_ok++;
                streams_.onParsedFrame(parsed);
                if (streams_.activeCount() == 0 &&
                    !parsed.packets.empty() && opus_dec_) {
                    std::vector<float> pcm_out;
                    opus_dec_->decode(parsed.packets[0].data.data(),
                                      parsed.packets[0].data.size(),
                                      pcm_out);
                    if (!pcm_out.empty() && playback_rings_[0]) {
                        playback_rings_[0]->write(pcm_out.data(),
                                                   pcm_out.size());
                    }
                }
            } else {
                live_stats_.frames_bad++;
                // Notify the coordinator of the loss so lost_run_ advances and
                // DRED / inband-FEC concealment can run on the next good frame.
                // The single-codeword and M/S branches already do this; the
                // asymmetric branch omitted it, silently disabling recovery in
                // asymmetric hierarchical mode.
                streams_.onFrameLost();
            }
            // Surface HP/LP confidence even in asymmetric mode: same
            // codeword carries both layers' bits, so the single decoder
            // result reflects both. Mark hier_active so GUI can show it.
            live_stats_.hier_active     = true;
            live_stats_.hp_avg_llr_mag  = dec_res.avg_magnitude;
            live_stats_.lp_avg_llr_mag  = dec_res.avg_magnitude;
            live_stats_.hp_locked       = parsed.crc_ok;
            live_stats_.lp_locked       = parsed.crc_ok;
            updateRxMeasurements();   // refresh SNR/EVM/CFO/PPM/BER (#9)
            continue;
        }

        // Demodulate each OFDM symbol → soft LLRs (single-codeword path)
        std::vector<float> all_llrs;

        // Accumulate constellation points across OFDM symbols and push
        // ONCE at the end of the codeword (avoids ~10 lock acquisitions
        // per tick that contend with the GUI paint thread).
        ComplexBuf cw_constellation_accum;
        cw_constellation_accum.reserve(rx_sym_len_ * 2);

        // BICM-ID needs the equalized symbols themselves (not just
        // LLRs) so the iterative demap can be re-run with prior LLR
        // feedback. We accumulate these only when BICM-ID is active to
        // avoid the alloc when it's off.
        const bool bicm_id_active = (ecfg_.use_bicm_id &&
                                      bicm_decoder_ &&
                                      ecfg_.bicm_outer_iter > 1);
        ComplexBuf bicm_syms;
        if (bicm_id_active) bicm_syms.reserve(rx_sym_len_ * 4);

        for (size_t base = 0; base + rx_sym_len_ <= cw_samples.size();
             base += rx_sym_len_) {
            ComplexBuf one_sym(
                cw_samples.begin() + static_cast<ptrdiff_t>(base),
                cw_samples.begin() + static_cast<ptrdiff_t>(base + rx_sym_len_));

            // Soft demodulation for FEC. ONE demodulator pass per symbol:
            // the eq_out parameter hands back the equalized symbols for the
            // constellation display, so neither branch re-runs demodulate()
            // (the old DD branch did — re-running the FFT and double-
            // advancing the CFO derotation NCO, MMSE, phase tracker and SRO
            // estimator: with a nonzero preamble CFO estimate the extra
            // sym_len NCO advance per symbol re-introduced the very offset
            // the Moose estimator had corrected). Both branches demap
            // through the demodulator's per-bin |H|^2 noise weighting —
            // the scalar-noise local demap previously used here forfeited
            // the per-bin gain on every frequency-selective channel. (#54)
            std::vector<float> llrs;
            ComplexBuf data_syms;
            if (ecfg_.use_dd_chest) {
                ofdm_demod_->demodulateSoftDD(one_sym, llrs, noise_var,
                                              ecfg_.use_pwl_llr, &data_syms);
            } else {
                ofdm_demod_->demodulateSoft(one_sym, llrs, noise_var,
                                            ecfg_.use_pwl_llr, &data_syms);
            }

            // Accumulate equalized symbols for the batched constellation
            // push (and for BICM-ID, which re-demaps with prior LLRs).
            if (!data_syms.empty()) {
                cw_constellation_accum.insert(cw_constellation_accum.end(),
                                              data_syms.begin(),
                                              data_syms.end());
                if (bicm_id_active) {
                    bicm_syms.insert(bicm_syms.end(),
                                      data_syms.begin(),
                                      data_syms.end());
                }
            }
            all_llrs.insert(all_llrs.end(), llrs.begin(), llrs.end());
        }

        // Batched constellation push — one lock per codeword.
        if (!cw_constellation_accum.empty()) {
            std::lock_guard<std::mutex> lock(state_.mtx);
            state_.constellation.push(cw_constellation_accum);
        }

        // Pad or truncate to codeword length
        all_llrs.resize(n_cw_, 0.f);

        // Deinterleave
        std::vector<float> deintlv(n_cw_);
        interleaver_->deinterleave(all_llrs.data(), deintlv.data());

        // Pre-FEC (uncoded) BER: hard-decision the LLRs, count syndrome
        // unsatisfied checks, and approximate BER from the parity-check ratio.
        // Useful for distinguishing channel SNR issues (high pre-BER) from
        // decoder problems (low pre-BER, high post-FER).
        if (ldpc_dec_bp_) {
            const auto& H = ldpc_dec_bp_->matrix();
            // Hard decisions
            std::vector<uint8_t> hard((H.n + 7) / 8, 0);
            for (size_t i = 0; i < H.n && i < deintlv.size(); ++i) {
                if (deintlv[i] < 0.f) {
                    hard[i >> 3] |= static_cast<uint8_t>(1u << (7 - (i & 7)));
                }
            }
            size_t unsat = 0;
            for (size_t c = 0; c < H.m; ++c) {
                uint8_t s = 0;
                for (uint32_t v : H.rows[c]) {
                    s ^= (hard[v >> 3] >> (7 - (v & 7))) & 1;
                }
                if (s) ++unsat;
            }
            // Heuristic: pre-FEC BER ≈ unsat / (m × avg_check_degree)
            float avg_deg = 0.f;
            for (size_t c = 0; c < H.m; ++c) avg_deg += static_cast<float>(H.rows[c].size());
            avg_deg /= std::max<size_t>(1, H.m);
            float pre_ber = (avg_deg > 0.f && H.m > 0)
                ? static_cast<float>(unsat) / (avg_deg * static_cast<float>(H.m))
                : 0.f;
            // Smooth so the readout doesn't flicker
            live_stats_.pre_fec_ber =
                0.9f * live_stats_.pre_fec_ber + 0.1f * pre_ber;
        }

        // LDPC decode (use configured decoder)
        std::vector<uint8_t> decoded_info(k_bytes_, 0);
        LDPCDecodeResult dec_result;

        if (bicm_id_active && !bicm_syms.empty()) {
            // BICM-ID overrides ORBGRAND / BP — the orchestrator drives
            // its own LDPC inner loop with extrinsic LLR feedback. Reuse
            // the equalized symbols accumulated above; pad/truncate to
            // codeword bits handled inside BICMDecoder.
            //
            // Update outer-iter count in case the user changed it
            // between codewords (no need to rebuild bicm_decoder_).
            BICMConfig bc = bicm_decoder_->config();
            bc.outer_iterations = std::max<size_t>(1, ecfg_.bicm_outer_iter);
            bicm_decoder_->setConfig(bc);
            dec_result = bicm_decoder_->decodeIterative(
                bicm_syms, noise_var, decoded_info.data());
        } else if (ecfg_.use_orbgrand && ldpc_dec_orb_) {
            dec_result = ldpc_dec_orb_->decode(deintlv.data(), decoded_info.data());
            // GRAND-assisted hybrid: when ORBGRAND exhausts its query
            // budget it returns the uncorrected hard decision
            // (converged=false). The layered min-sum BP decoder is always
            // allocated and idle — fall through to it and keep whichever
            // result actually converges, instead of dropping a frame BP
            // could have recovered. (Issue #14.)
            if (!dec_result.converged && ldpc_dec_bp_) {
                std::vector<uint8_t> bp_info(k_bytes_, 0);
                auto bp_result = ldpc_dec_bp_->decode(deintlv.data(),
                                                      bp_info.data());
                if (bp_result.converged) {
                    decoded_info = std::move(bp_info);
                    dec_result   = bp_result;
                }
            }
        } else if (ldpc_dec_bp_) {
            dec_result = ldpc_dec_bp_->decode(deintlv.data(), decoded_info.data());
        }

        live_stats_.frames_rx++;

        bool frame_ok = false;   // for OLLA outer-loop link adaptation
        if (dec_result.converged) {
            // Reed-Solomon outer-code correction: strip residual byte
            // errors that survived the LDPC waterfall. Falls through
            // unchanged when RS is disabled or the block size is
            // outside the 255-byte RS window.
            bool rs_active_rx = (modem_p_.enable_rs_outer &&
                                  k_bytes_ <= ReedSolomon::MAX_BLOCK &&
                                  k_bytes_ > ReedSolomon::PARITY_BYTES + 1);
            if (rs_active_rx) {
                size_t rs_data_len = k_bytes_ - ReedSolomon::PARITY_BYTES;
                std::vector<uint8_t> rs_block(k_bytes_, 0);
                std::memcpy(rs_block.data(), decoded_info.data(),
                            std::min(decoded_info.size(), k_bytes_));
                int n_corrected = rs_.decode(rs_block.data(), rs_data_len);
                if (n_corrected >= 0) {
                    // Resize decoded_info to just the data portion
                    // (parity bytes stripped) for FrameParser.
                    decoded_info.assign(rs_block.begin(),
                                         rs_block.begin() + static_cast<ptrdiff_t>(rs_data_len));
                }
                // n_corrected < 0 means RS couldn't fix the block; pass
                // the raw LDPC output to the parser anyway (CRC will
                // catch any remaining corruption).
            }

            // Parse frame
            ParsedFrame parsed;
            FrameParser::parse(decoded_info, parsed);

            if (parsed.valid && parsed.crc_ok) {
                live_stats_.frames_ok++;
                frame_ok = true;

                // Round-trip latency: look up the TX timestamp of the frame we
                // actually just decoded (keyed by its frame_number), not the
                // current tick's just-stamped frame. The old code read the slot
                // TX wrote microseconds earlier in the SAME tick, so it measured
                // the intra-tick TX->RX delta (~0 ms) instead of the true
                // in-flight latency across the accumulator/loopback.
                auto tx_t = pending_tx_times_[parsed.frame_number % PENDING_RING];
                double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - tx_t).count();
                if (ms > 0 && ms < 2000) {
                    live_stats_.rt_latency_ms =
                        0.85f * live_stats_.rt_latency_ms + 0.15f * static_cast<float>(ms);
                }

                // Multi-stream dispatch: each parsed packet has a stream_id;
                // hand it to the matching per-stream Opus decoder. The
                // coordinator's onParsedFrame writes decoded PCM into each
                // stream's RX ring (drained by the playback monitor device).
                streams_.onParsedFrame(parsed);

                // Legacy single-stream playback fallback for backward compat
                // when no streams are enabled (only stream 0 / packet 0).
                // Route to ring 0 so it appears on output channel 0 (Left).
                if (streams_.activeCount() == 0 &&
                    !parsed.packets.empty() && opus_dec_) {
                    std::vector<float> pcm_out;
                    opus_dec_->decode(parsed.packets[0].data.data(),
                                      parsed.packets[0].data.size(),
                                      pcm_out);
                    if (!pcm_out.empty() && playback_rings_[0]) {
                        playback_rings_[0]->write(pcm_out.data(),
                                                   pcm_out.size());
                    }
                }
            } else {
                live_stats_.frames_bad++;
                // CRC failed after LDPC convergence — mark every enabled
                // stream as having lost this frame so the next frame's
                // packets can attempt Opus inband-FEC recovery.
                streams_.onFrameLost();
            }
        } else {
            live_stats_.frames_bad++;

            // LDPC failed to converge: all streams lost this frame. The
            // next received frame's packets carry inband FEC that can
            // reconstruct the missing audio (Opus encoder configured
            // with OPUS_SET_INBAND_FEC=1).
            streams_.onFrameLost();

            // Legacy single-stream PLC (when no streams are enabled).
            if (opus_dec_) {
                std::vector<float> plc_out;
                opus_dec_->decodePLC(plc_out);
            }
        }

        // Update BER + SNR/EVM/CFO/PPM from the demodulator. Hoisted into a
        // helper so the M/S and asymmetric-hierarchical branches (which
        // `continue` before reaching here) refresh the same telemetry —
        // previously those modes froze SNR/EVM/CFO/PPM/BER readouts. (#9)
        updateRxMeasurements();

        // ---- AMC live loop ----
        // Feed measured SNR into the selector, accept its ModCod
        // recommendation, and write it back into AppState. The PLS
        // signaling path (already present in the engine) carries the
        // updated ModCod to the receiver via the next preamble + PLS frame.
        if (ecfg_.use_amc) {
            // Feed this frame's decode outcome into OLLA so the selector
            // self-calibrates its SNR bias to the target error rate. (SOTA #4)
            amc_.reportFrameResult(frame_ok);
            AMCEntry rec = amc_.recommend(live_stats_.snr_db,
                                          ofdm_p_.modulation,
                                          frame_p_.fec_rate);
            if (rec.mod != ofdm_p_.modulation || rec.fec != frame_p_.fec_rate) {
                {
                    std::lock_guard<std::mutex> lock(state_.mtx);
                    state_.ofdm.modulation = rec.mod;
                    state_.frame.fec_rate  = rec.fec;
                    state_.preset_modified = true;
                }
                // Trigger reconfiguration on next tick
                onConfigChanged();
            }
        }
    }

    // Drive the sync FSM from the REAL decode outcomes of the codewords just
    // processed this tick (all branches bump frames_ok / frames_bad). Only act
    // when codewords were actually processed — a tick with no full codeword
    // leaves the FSM in its current state rather than flapping to Lost. On
    // sustained failure after a prior lock, drop the preamble/PLS latches so
    // the next tick re-acquires (and the search block above re-correlates).
    {
        const auto ok_d  = live_stats_.frames_ok  - ok_before;
        const auto bad_d = live_stats_.frames_bad - bad_before;
        if (ok_d + bad_d > 0) {
            if (ok_d > 0) {
                live_stats_.sync_state =
                    sync_fsm_.feed(true, 1.0f, live_stats_.ber_estimate);
                consec_bad_ticks_ = 0;
                had_sync_ = true;
            } else {
                live_stats_.sync_state = sync_fsm_.feed(false, 0.0f, 1.0f);
                if (had_sync_ && ++consec_bad_ticks_ >= RESYNC_FAIL_TICKS) {
                    // Drop the latch → re-enter acquisition next tick, which
                    // re-correlates to the next preamble and resets
                    // rx_frame_count_ for fresh per-frame alignment.
                    preamble_received_ = false;
                    rx_pls_pending_    = false;
                    consec_bad_ticks_  = 0;
                }
            }
        }
    }

    // Drain per-stream decoded audio:
    //   - For mono streams: Mid PCM → playback_rings_[id] (single channel)
    //   - For stereo streams in M/S hierarchical mode: recombine L = Mid
    //     + α·Side + (1−α)·SideRecon, R = Mid − (those side terms), with
    //     α derived from the LP-layer LDPC LLR confidence — so a strong
    //     LP-decode plays the transmitted Side exactly, a marginal one
    //     crossfades to the SideReconstructor's synthesized Side, and a
    //     failed LP gives a fully-decorrelated synthetic Side. Writes:
    //       L → playback_rings_[id]  (channel id)
    //       R → playback_rings_[id + MAX_STREAMS]  (channel id+8)
    //   - Tee a copy into each stream's WAV recorder (Mid only).
    if (streams_.activeCount() > 0) {
        constexpr size_t CHUNK = 2048;
        float mid_scratch[CHUNK];
        float side_scratch[CHUNK];

        // LP-layer crossfade weight: 1.0 = pure transmitted Side, 0.0 =
        // pure synthesized Side. Updated from the latest LP LDPC stats.
        // Outside M/S mode this stays at 1.0 (irrelevant — stereo path
        // doesn't activate for non-hier streams anyway).
        float side_alpha = 1.0f;
        if (ms_mode_active_) {
            float conf = SideReconstructor::confidenceFromLLR(
                live_stats_.lp_avg_llr_mag);
            // Drop to 0 immediately when LP fails entirely; otherwise
            // smooth confidence-derived blend.
            side_alpha = live_stats_.lp_locked ? conf : 0.0f;
        }

        for (size_t id = 0; id < MAX_STREAMS; ++id) {
            size_t avail = streams_.rxAvailable(id);
            if (avail == 0) continue;
            size_t want = std::min<size_t>(CHUNK, avail);
            size_t got  = streams_.pullRX(id, mid_scratch, want);
            if (got == 0) continue;

            // Tee 1: per-stream WAV recorder (Mid PCM — recorders are
            // documented as mono). Guard against a concurrent
            // start/stopStreamRecording() on the GUI thread fclose'ing the
            // FILE between the null-check and the fwrite.
            {
                std::lock_guard<std::mutex> lk(io_mtx_);
                if (stream_recorders_[id] && stream_recorders_[id]->f) {
                    auto& r = *stream_recorders_[id];
                    std::vector<int16_t> i16(got);
                    for (size_t k = 0; k < got; ++k) {
                        float v = std::max(-1.f, std::min(1.f, mid_scratch[k]));
                        i16[k] = static_cast<int16_t>(v * 32767.f);
                    }
                    std::fwrite(i16.data(), sizeof(int16_t), got, r.f);
                    r.frames_written += got;
                }
            }

            const bool stereo = ms_mode_active_ && side_recons_[id];
            if (!stereo) {
                // Mono stream — single ring at index id. Clamp the Mid
                // samples and route as the channel-id output.
                if (playback_rings_[id]) {
                    for (size_t k = 0; k < got; ++k) {
                        mid_scratch[k] = std::max(-1.f, std::min(1.f,
                                                  mid_scratch[k]));
                    }
                    playback_rings_[id]->write(mid_scratch, got);
                }
                continue;
            }

            // Stereo path — pull as much Side as is buffered (capped at
            // the same `got` we drained from Mid). Pad with 0 if Side
            // is behind Mid (transient: Side decoder hasn't produced a
            // matching frame yet). The synthesizer fills the gap so
            // there's always a stereo image.
            std::fill(side_scratch, side_scratch + got, 0.f);
            size_t side_avail = streams_.sideRxAvailable(id);
            size_t side_n = std::min<size_t>(got, side_avail);
            if (side_n > 0) {
                streams_.pullSideRX(id, side_scratch, side_n);
            }

            // Synthesize Side from Mid for the same span — even when
            // we have transmitted Side, the reconstructor runs each
            // frame so its IIR state stays warm and continuous (no
            // transient when LP later drops out).
            std::vector<float> synth_side;
            side_recons_[id]->process(mid_scratch, got, synth_side);
            if (synth_side.size() < got) synth_side.resize(got, 0.f);

            // Blend transmitted vs synthesized Side per the LP-confidence
            // crossfade weight; recombine L/R.
            const float a = side_alpha;
            for (size_t k = 0; k < got; ++k) {
                float side = a * side_scratch[k] +
                             (1.f - a) * synth_side[k];
                float L = mid_scratch[k] + side;
                float R = mid_scratch[k] - side;
                // Hard-clamp before handing off to the audio callback.
                if (L >  1.f) L =  1.f; else if (L < -1.f) L = -1.f;
                if (R >  1.f) R =  1.f; else if (R < -1.f) R = -1.f;
                mid_scratch[k]  = L;
                side_scratch[k] = R;
            }
            if (playback_rings_[id]) {
                playback_rings_[id]->write(mid_scratch, got);
            }
            const size_t r_ring = id + MAX_STREAMS;
            if (r_ring < PLAYBACK_RING_COUNT && playback_rings_[r_ring]) {
                playback_rings_[r_ring]->write(side_scratch, got);
            }
        }
    }
}

// =========================================================================
// IQ record / playback
// =========================================================================

// =========================================================================
// Per-stream decoded-audio WAV recording
// =========================================================================

namespace {
inline void wavPut32(std::FILE* f, uint32_t x) {
    uint8_t b[4] = { static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8),
                     static_cast<uint8_t>(x >> 16), static_cast<uint8_t>(x >> 24) };
    std::fwrite(b, 1, 4, f);
}
inline void wavPut16(std::FILE* f, uint16_t x) {
    uint8_t b[2] = { static_cast<uint8_t>(x), static_cast<uint8_t>(x >> 8) };
    std::fwrite(b, 1, 2, f);
}
void writeWavHeader(std::FILE* f, uint32_t sr, uint16_t ch,
                     uint32_t data_bytes) {
    std::fseek(f, 0, SEEK_SET);
    std::fwrite("RIFF", 1, 4, f);
    wavPut32(f, 36 + data_bytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    wavPut32(f, 16);
    wavPut16(f, 1);              // PCM
    wavPut16(f, ch);
    wavPut32(f, sr);
    wavPut32(f, sr * ch * 2);    // byte rate
    wavPut16(f, ch * 2);         // block align
    wavPut16(f, 16);             // bits/sample
    std::fwrite("data", 1, 4, f);
    wavPut32(f, data_bytes);
}
} // anonymous

bool AudioEngine::startStreamRecording(size_t id, const std::string& path) {
    if (id >= MAX_STREAMS) return false;
    stopStreamRecording(id);
    auto rec = std::make_unique<WavRecorder>();
    rec->f = std::fopen(path.c_str(), "wb");
    if (!rec->f) return false;
    rec->path        = path;
    // Decoded stream PCM is at the fixed audio rate, NOT the RF rate —
    // stamping ofdm_p_.sample_rate made 96/192 kHz-preset recordings play
    // 2-4x fast (and was an unlocked cross-thread read of engine state).
    rec->sample_rate = kAudioSampleRate;
    rec->channels    = 1;
    writeWavHeader(rec->f, rec->sample_rate, rec->channels, 0);
    std::lock_guard<std::mutex> lk(io_mtx_);   // engine thread reads this slot
    stream_recorders_[id] = std::move(rec);
    return true;
}

void AudioEngine::stopStreamRecording(size_t id) {
    if (id >= MAX_STREAMS) return;
    std::lock_guard<std::mutex> lk(io_mtx_);
    if (!stream_recorders_[id]) return;
    auto& r = *stream_recorders_[id];
    if (r.f) {
        uint32_t data_bytes = static_cast<uint32_t>(r.frames_written
                                * r.channels * 2);
        writeWavHeader(r.f, r.sample_rate, r.channels, data_bytes);
        std::fclose(r.f);
        r.f = nullptr;
    }
    stream_recorders_[id].reset();
}

bool AudioEngine::isStreamRecording(size_t id) const {
    if (id >= MAX_STREAMS) return false;
    std::lock_guard<std::mutex> lk(io_mtx_);
    return stream_recorders_[id] && stream_recorders_[id]->f;
}

bool AudioEngine::startIQRecording(const std::string& path) {
    return iq_recorder_.open(path,
        ofdm_p_.sample_rate ? ofdm_p_.sample_rate : 48000u);
}

void AudioEngine::stopIQRecording() {
    iq_recorder_.close();
}

bool AudioEngine::startIQPlayback(const std::string& path) {
    // open() reallocates the sample buffer + resets pos_; the engine thread
    // reads iq_player_ in processRX, so this must be serialized (IQPlayer has
    // no internal locking).
    std::lock_guard<std::mutex> lk(io_mtx_);
    if (!iq_player_.open(path)) return false;
    iq_player_active_.store(true);
    return true;
}

std::vector<uint32_t> AudioEngine::supportedSampleRates() const {
    // Always create a fresh local HWAudioDevice for this query, so we don't
    // race with the engine thread's `hw_audio_` lifecycle (init/teardown).
    // The local instance is destroyed when this function returns.
    HWAudioDevice probe;
    if (!probe.init()) return {};
    return probe.supportedSampleRates(
        ecfg_.playback_device, ecfg_.capture_device);
}

std::vector<AudioDeviceInfo> AudioEngine::enumerateCaptureDevices() const {
    // Fresh probe (see supportedSampleRates rationale).
    HWAudioDevice probe;
    if (!probe.init()) return {};
    return probe.captureDevices();
}

} // namespace gw
