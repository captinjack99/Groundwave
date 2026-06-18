/**
 * @file audio_engine.hpp
 * @brief Threaded audio engine — owns and runs the full TX/RX DSP chain
 *
 * Runs in a dedicated QThread, processing OFDM frames through the
 * complete signal chain:
 *
 *   TX: test tone/PCM → Opus encode → Frame build → LDPC encode
 *       → interleave → OFDM modulate → SoundcardModem::transmit()
 *
 *   RX: SoundcardModem::receive() → sync → OFDM demodulate
 *       → deinterleave → LDPC decode → Frame parse → Opus decode → PCM
 *
 * The engine feeds live measurement data back to AppState for GUI display:
 *   - Spectrum samples (passband or baseband)
 *   - Constellation points (equalized data symbols)
 *   - ModemStats (SNR, EVM, BER, frame counters, sync state)
 *   - Level meter data (TX/RX RMS, peak, clip)
 *   - AGC gain
 *
 * Thread safety:
 *   - Engine owns all DSP objects (no sharing)
 *   - Writes to AppState via mutex-protected setters
 *   - DataBridge::pushSpectrumSamples() is the only cross-thread call
 *     to the bridge (the bridge's SpectrumAnalyzer has its own lock)
 *   - Configuration changes (modcod, enable, etc.) are read from AppState
 *     under lock and applied between frames
 */
#pragma once

#include "../include/app_state.hpp"
#include "../include/ofdm.hpp"
#include "../include/ldpc.hpp"
#include "../include/orbgrand.hpp"
#include "../include/interleaver.hpp"
#include "../include/frame.hpp"
#include "../include/opus_codec.hpp"
#include "../include/soundcard_modem.hpp"
#include "../include/hw_audio.hpp"
#include "../include/papr_reducer.hpp"
#include "../include/hierarchical_mod.hpp"
#include "../include/pls.hpp"
#include "../include/sync_fsm.hpp"
#include "../include/amc.hpp"
#include "../include/multi_stream.hpp"
#include "../include/reed_solomon.hpp"
#include "../include/side_reconstructor.hpp"
#include "../include/bicm.hpp"
#include "../include/sfo_resampler.hpp"
#include "../include/iq_io.hpp"
#include "data_bridge.hpp"

#include <QObject>
#include <QThread>
#include <QTimer>
#include <array>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>

namespace gw {

// =========================================================================
// Audio Engine Configuration
// =========================================================================

struct AudioEngineConfig {
    bool   use_orbgrand   = true;    ///< Use ORBGRAND instead of BP decoder
                                     ///< (near-ML for short high-rate codes,
                                     ///<  Duffy/Médard 2022-2024 SOTA)
    bool   use_mmse       = false;   ///< Use MMSE channel estimation
    bool   use_pwl_llr    = true;    ///< Use piecewise-linear LLR (faster soft demap)
    bool   use_dd_chest   = false;   ///< Decision-directed channel-estimate refinement
                                     ///< (per-symbol DD refine + re-equalize → ~0.3 dB)
    bool   use_bicm_id    = false;   ///< Iterative BICM (demap ↔ decode w/ extrinsic LLR
                                     ///< feedback). Trades CPU for 1–2 dB at marginal SNR.
    size_t bicm_outer_iter = 3;      ///< Outer iterations for BICM-ID (1 = standard BICM)
    bool   bicm_inner_sogrand = false; ///< Inner soft-output decoder for BICM-ID:
                                     ///< false = BP (LDPCDecoder), true = SOGRAND
                                     ///< (ORBGRANDDecoder). Default OFF (BP): measured
                                     ///< BP-inner strictly beats SOGRAND-inner at every
                                     ///< tested modcod/SNR — List-GRAND's posterior is
                                     ///< over only L=8 candidates, so at these short
                                     ///< low/medium-rate blocks the true codeword is
                                     ///< usually outside the list near the cliff, whereas
                                     ///< BP carries full-graph marginals. Opt-in only.
    bool   use_hw_audio   = false;   ///< Use hardware audio (vs loopback)
    float  loopback_snr   = -1.f;    ///< <0 = perfect loopback, >=0 = add AWGN
    bool   generate_tone  = true;    ///< Generate test tone for TX (vs silence)
    float  tone_freq_hz   = 1000.f;  ///< Test tone frequency
    float  tone_amplitude = 0.5f;    ///< Test tone amplitude
    size_t process_interval_ms = 10; ///< Processing loop interval
    int    playback_device = -1;     ///< HW playback device (-1 = default)
    int    capture_device  = -1;     ///< HW capture device (-1 = default)

    // Phase 5 advanced features
    PAPRConfig          papr;        ///< PAPR reduction config
    HierarchicalConfig  hier;        ///< Hierarchical modulation config
    VCMSchedule         vcm;         ///< VCM superframe schedule

    // Adaptive Modulation & Coding
    bool                use_amc      = false; ///< Enable AMC live loop
    AMCConfig           amc;
};

// =========================================================================
// Audio Engine
// =========================================================================

class AudioEngine : public QObject {
    Q_OBJECT

public:
    AudioEngine(AppState& state, DataBridge& bridge, QObject* parent = nullptr);
    ~AudioEngine() override;

    /** Start recording the decoded audio from stream `id` to a 16-bit PCM
     *  WAV file. Returns true if the file opened successfully.
     *  Stop with stopStreamRecording(id). */
    bool startStreamRecording(size_t id, const std::string& path);
    void stopStreamRecording(size_t id);
    bool isStreamRecording(size_t id) const;

    /// Start the engine in its own thread
    void startup();

    /// Stop the engine and wait for thread to finish
    void shutdown();

    /// Check if currently running
    bool isRunning() const { return running_.load(); }

    /// Access config (apply changes via setEngineConfig). Returns a COPY under
    /// ecfg_mtx_ — the previous `const&` let the GUI thread read ecfg_ (which
    /// embeds std::vectors in vcm/papr/amc) while the engine thread wrote it in
    /// setEngineConfig's queued lambda, a data race that could read a vector
    /// mid-reallocation. The copy is cheap relative to its (infrequent,
    /// GUI-thread, display/dialog) call sites.
    AudioEngineConfig engineConfig() const {
        std::lock_guard<std::mutex> lk(ecfg_mtx_);
        return ecfg_;
    }

    /// Access stream-0 playback ring buffer (legacy entry point; the
    /// per-stream rings are routed internally to audio_monitor_).
    RingBuffer* playbackRing() {
        return playback_rings_[0] ? playback_rings_[0].get() : nullptr;
    }

    /// Query the supported sample rates for the currently selected HW
    /// audio devices. Returns empty if HW audio isn't available or the
    /// engine isn't running. Used by the TX panel to populate its SR combo
    /// with rates the device actually supports (vs. the hardcoded list).
    std::vector<uint32_t> supportedSampleRates() const;

    /// Enumerate available capture (microphone / line-in) devices.
    /// Spins up a temporary HWAudioDevice just for enumeration if no
    /// engine-owned device exists yet. Used by the Stream Panel to
    /// populate per-stream input-device pickers.
    std::vector<AudioDeviceInfo> enumerateCaptureDevices() const;

public slots:
    /// Called when the user changes configuration in the GUI
    void onConfigChanged();

    /// Set engine-specific options
    void setEngineConfig(const AudioEngineConfig& cfg);

    /// IQ recording control. Captures the post-downconvert RX baseband
    /// to an interleaved float32 WAV (channel 0=I, channel 1=Q).
    bool startIQRecording(const std::string& path);
    void stopIQRecording();
    bool isIQRecording() const { return iq_recorder_.isOpen(); }

    /// IQ playback: load a recorded WAV and feed it as the RX input on
    /// the next ticks. Use to replay captures for offline debugging.
    bool startIQPlayback(const std::string& path);
    void stopIQPlayback() { iq_player_active_.store(false); }

signals:
    /// Emitted when the engine starts/stops processing
    void engineStarted();
    void engineStopped();

    /// Emitted on fatal error (e.g., codec init failure)
    void engineError(const QString& message);

private slots:
    /// Main processing tick — called by timer in engine thread
    void processTick();

private:
    // ---- Initialization (called in engine thread) ----
    void initDSP();
    void teardownDSP();

    // ---- TX path ----
    void processTX();
    void generateTestAudio(float* pcm, size_t samples);

    // ---- RX path ----
    void processRX();
    /** Refresh BER/SNR/EVM/CFO/PPM from the demodulator. Called at the end
     *  of every RX decode branch (single-codeword, M/S, asymmetric) so the
     *  telemetry doesn't freeze in hierarchical modes. */
    void updateRxMeasurements();

    // ---- RX PLS detection ----
    void processRXPLS();

    // ---- VCM per-slot FEC rebuild ----
    void rebuildFEC(FECRate rate, Modulation mod);

    // ---- Config sync ----
    void syncConfig();

    // ---- State refs ----
    AppState&   state_;
    DataBridge& bridge_;

    // ---- Thread ----
    QThread     thread_;
    QTimer*     timer_ = nullptr;
    std::atomic<bool> running_{false};
    /// Set by shutdown(); the QThread::started lambda checks it so a stop
    /// issued during the startup window skips bring-up instead of racing
    /// the pending quit() (which wedged the engine for the session).
    std::atomic<bool> shutdown_requested_{false};
    /// Rebuild-coalescing generation: onConfigChanged() queues one DSP
    /// rebuild per call, but only the newest needs to run — stale queued
    /// rebuilds check this and skip (a burst of config clicks otherwise
    /// serializes seconds of teardown/init ahead of everything else).
    std::atomic<uint64_t> config_gen_{0};

    // ---- Engine config ----
    AudioEngineConfig ecfg_;
    // Guards ecfg_ against the GUI thread's engineConfig() copy racing the
    // engine thread's setEngineConfig write. The engine thread's own reads in
    // processTick are serialized with that write (same thread), so they need
    // no lock. Never held across state_.mtx.
    mutable std::mutex ecfg_mtx_;

    // ---- Cached copies of AppState config (read under lock) ----
    OFDMParams  ofdm_p_;
    FrameParams frame_p_;
    ModemConfig modem_p_;
    bool        tx_enabled_ = false;
    bool        prev_tx_enabled_ = false;  ///< prior-tick TX state (ramp-down edge detect)
    float       tx_gain_db_ = 0.f;

    // ---- DSP components (owned, created in engine thread) ----
    std::unique_ptr<SoundcardModem>   modem_;
    std::unique_ptr<HWAudioDevice>    hw_audio_;       ///< modulated TX/RX duplex
    std::unique_ptr<HWAudioDevice>    audio_monitor_;  ///< plays decoded RX audio
    // ---- Mic capture (per-device) ----
    // Each unique `input_device` value among enabled Microphone-source
    // streams gets its own HWAudioDevice and ring buffer. Streams with
    // the same input_device share the ring. Key -1 means system default.
    struct MicCapture {
        std::unique_ptr<HWAudioDevice> dev;
        std::unique_ptr<RingBuffer>    ring;
        std::string                    last_pcm_label;  // for stderr logs
    };
    std::map<int, MicCapture>          mic_captures_;
    std::unique_ptr<OFDMModulator>    ofdm_mod_;
    std::unique_ptr<OFDMDemodulator>  ofdm_demod_;
    std::unique_ptr<OFDMSynchronizer> sync_;
    std::unique_ptr<LDPCEncoder>      ldpc_enc_;
    std::unique_ptr<LDPCDecoder>      ldpc_dec_bp_;
    std::unique_ptr<ORBGRANDDecoder>  ldpc_dec_orb_;
    std::unique_ptr<BitInterleaver>   interleaver_;
    // BICM iterative-demap orchestrator. Borrows mapper / interleaver
    // / decoder pointers from the modcod-specific objects above. Rebuilt
    // in initDSP whenever the modcod changes, since the symbol mapper
    // and interleaver geometry depend on modulation + FEC rate.
    std::unique_ptr<SymbolMapper>     bicm_mapper_;
    std::unique_ptr<BICMDecoder>      bicm_decoder_;
    bool bicm_orbgrand_note_shown_ = false;  ///< one-shot #53 note guard
    std::unique_ptr<OpusAudioEncoder> opus_enc_;
    std::unique_ptr<OpusAudioDecoder> opus_dec_;

    // ---- M/S hierarchical mode: parallel "Side" codec + LDPC chain ----
    // Used when hier_mapper_ is enabled AND the constellation has a symmetric
    // HP/LP bit split (hp_bps == lp_bps). The Side stream rides the LP layer
    // of the constellation while Mid rides HP, giving stereo over hierarchical
    // modulation with proper unequal error protection.
    std::unique_ptr<OpusAudioEncoder> side_enc_;
    std::unique_ptr<OpusAudioDecoder> side_dec_;
    std::unique_ptr<LDPCEncoder>      side_ldpc_enc_;
    std::unique_ptr<LDPCDecoder>      side_ldpc_dec_;
    std::unique_ptr<BitInterleaver>   side_interleaver_;
    bool                              ms_mode_active_ = false;

    // ---- Phase 5 DSP components ----
    std::unique_ptr<PAPRReducer>        papr_;
    std::unique_ptr<HierarchicalMapper> hier_mapper_;
    ModCodDetector                      modcod_det_{2};

    // ---- Frame processing state ----
    uint32_t frame_number_ = 0;
    bool     preamble_sent_ = false;
    bool     preamble_received_ = false;
    uint32_t preamble_interval_ = 50;  ///< Frames between preambles
    // Re-acquisition state. had_sync_ latches once we've ever decoded a good
    // frame; consec_bad_ticks_ counts consecutive ticks where codewords were
    // processed but none CRC-passed. After RESYNC_FAIL_TICKS of sustained
    // failure we drop the preamble latch and re-acquire — realigning to the
    // next retransmitted preamble via the synchronizer's correlation, so a
    // gross sample slip (HW-audio dropout) no longer offsets the FFT window
    // permanently. (The initial-acquisition path is unaffected: re-correlation
    // is gated on had_sync_.)
    bool     had_sync_         = false;
    int      consec_bad_ticks_ = 0;
    static constexpr int RESYNC_FAIL_TICKS = 40;  ///< ~0.4 s before re-acquire

    // ---- RX accumulation ----
    ComplexBuf rx_accum_;              ///< Accumulated RX baseband samples
    size_t     rx_sym_len_ = 0;       ///< OFDM symbol length (FFT + CP)
    size_t     rx_codeword_bits_ = 0; ///< Bits per LDPC codeword

    // ---- Sample-rate-offset (SFO) correction ----
    // Closes the SRO loop: the OFDM demod's pilot-slope estimate
    // (clockPpm()) drives this arbitrary-ratio resampler so two drifting
    // soundcard clocks stay symbol-aligned over long transmissions. Active
    // only when AFC is on (same gate as the SRO estimator). PI loop gains
    // validated in sfo_test.cpp.
    SFOResampler sfo_resampler_;
    bool         sfo_active_ = false;
    static constexpr float SFO_KP = -6.0f;
    static constexpr float SFO_KI = -0.5f;

    // ---- Measurement ----
    ModemStats live_stats_;
    float      tone_phase_ = 0.f;     ///< Test tone oscillator state

    // ---- Derived sizes ----
    size_t k_bytes_ = 0;  ///< LDPC info bytes
    size_t n_bytes_ = 0;  ///< LDPC codeword bytes
    size_t n_cw_    = 0;  ///< LDPC codeword bits
    size_t frame_capacity_ = 0; ///< Frame payload capacity (after RS overhead)

    // Reed-Solomon outer code. When `modem_p_.enable_rs_outer` is set,
    // the engine wraps the LDPC info block: payload occupies
    // (k_bytes - 16) bytes, the last 16 bytes are RS parity. On RX, the
    // LDPC-decoded info bytes go through `rs_.decode()` before being
    // handed to the FrameParser. RS corrects up to 8 byte errors per
    // block — useful for the residual-error region of the LDPC waterfall.
    ReedSolomon rs_;

    // ---- Audio playback ----
    // Per-stream playback rings, indexed by output channel:
    //   ring i              = stream i's LEFT channel (mono streams use
    //                          this as their only channel)
    //   ring i + MAX_STREAMS = stream i's RIGHT channel (stereo streams
    //                          only; idle for mono)
    // The audio_monitor_ device's multi-channel callback reads ring k
    // for output channel k. When the OS device has fewer channels than
    // PLAYBACK_RING_COUNT, the callback wraps via modulo (so a stereo
    // physical device gets L↔R alternation across the streams). When no
    // streams are enabled, ring 0 carries the legacy single-stream
    // fallback path.
    static constexpr size_t PLAYBACK_RING_COUNT = 2 * MAX_STREAMS;  // 16
    std::array<std::unique_ptr<RingBuffer>, PLAYBACK_RING_COUNT> playback_rings_;
    static constexpr size_t PLAYBACK_RING_SIZE = 48000; ///< ~1s at 48kHz

    // Per-stream Side reconstructors. Used in the drain stage to
    // synthesize Side from Mid when LP decode fails (graceful
    // degradation), and to crossfade with transmitted Side at marginal
    // LP confidence. One per stream so each stereo stream maintains
    // independent all-pass-cascade state.
    std::array<std::unique_ptr<SideReconstructor>, MAX_STREAMS> side_recons_;

    // mic_captures_ replaces the old single mic_capture_ring_ — see above.

    // Per-stream file playback state (one looping WAV reader per stream
    // whose source is set to File). Loaded lazily in syncConfig and torn
    // down when the source changes away from File.
    struct FileSource {
        std::vector<float> samples;     // mono, float
        size_t             pos = 0;
        std::string        path;
        uint32_t           sample_rate = 48000;
    };
    std::array<std::unique_ptr<FileSource>, MAX_STREAMS> file_sources_{};

    // Per-stream WAV recorders for decoded RX audio. NULL when not recording.
    struct WavRecorder {
        std::FILE* f = nullptr;
        uint32_t   sample_rate = 48000;
        uint16_t   channels    = 1;
        uint64_t   frames_written = 0;
        std::string path;
    };
    std::array<std::unique_ptr<WavRecorder>, MAX_STREAMS> stream_recorders_{};

    // ---- VCM per-slot FEC tracking ----
    Modulation tx_vcm_last_mod_ = Modulation::QPSK;  ///< Last slot modulation
    FECRate    tx_vcm_last_fec_ = FECRate::Rate_1_2;  ///< Last slot FEC rate

    // ---- RX PLS auto-detect state ----
    bool       pls_received_ = false;   ///< PLS decoded at least once (acquired)
    // RX consumes one PLS OFDM symbol per data codeword (TX transmits a PLS
    // symbol every frame) and one preamble every preamble_interval-th frame —
    // this counter tracks which frame we're on since (re)acquisition so the
    // periodic preamble is skipped at the right cadence. Reset to 0 whenever a
    // preamble is (re)acquired.
    size_t     rx_frame_count_ = 0;
    size_t     pls_samples_needed_ = 0; ///< Samples for the PLS block (>=1 OFDM symbol)
    /// This frame's preamble+PLS were consumed but its codeword (possibly
    /// re-sized by a PLS-driven FEC rebuild) wasn't buffered yet — resume at
    /// the codeword slice next tick instead of re-stripping preamble/PLS.
    bool       rx_pls_pending_ = false;

    // ---- Symbol-timing tracking (CP early/late gate, OFDMSynchronizer) ----
    // Once per codeword the demod runs trackTiming() on the next symbol and
    // applies its -1/0/+1 nudge to the FFT-window position, holding alignment
    // against soundcard sample-clock drift over a long burst. DEFAULT ON: the
    // loop is a ±1-sample, slow-accumulator gate (gain 0.05) so on a drift-free
    // stream it returns 0 and is a no-op; the integration suite confirms no
    // regression. Disable-able for diagnostics / A-B.
    bool       timing_track_enabled_ = true;

    // ---- Sync FSM (Searching → Acquiring → Locked → Tracking → Lost) ----
    SyncFSM    sync_fsm_;

    // ---- AMC selector (drives ModCod adaptation from measured SNR) ----
    AMCSelector amc_;

    // ---- Multi-stream coordinator (per-stream Opus enc/dec + ring buffers) ----
    MultiStreamCoordinator streams_;

    // ---- Round-trip latency tracking (TX→RX in loopback) ----
    static constexpr size_t PENDING_RING = 256;
    std::array<std::chrono::steady_clock::time_point, PENDING_RING>
        pending_tx_times_{};

    // ---- AGC pumping detector: rolling window of gain_db deltas ----
    static constexpr size_t AGC_WINDOW = 64;
    std::array<float, AGC_WINDOW> agc_history_{};
    size_t                        agc_history_pos_ = 0;
    size_t                        agc_history_filled_ = 0;  ///< valid entries so far

    // ---- IQ record / playback ----
    IQRecorder         iq_recorder_;
    IQPlayer           iq_player_;
    std::atomic<bool>  iq_player_active_{false};

    // Guards the file-I/O resources (per-stream WAV recorders + the IQ player)
    // that are toggled from the GUI thread (startStreamRecording / IQ playback
    // control) while the engine thread reads/writes them in processRX. Without
    // this, a stop on the GUI thread could fclose/realloc a buffer between the
    // engine thread's null-check and its write → use-after-free. Distinct from
    // state_.mtx; never held across a state_.mtx acquisition.
    mutable std::mutex io_mtx_;

    // ---- Tick-latency instrumentation (lightweight) ----
    // Tracks the longest processTick wall-clock time over the last
    // TICK_LATENCY_WINDOW ticks. Surfaced via ModemStats so the GUI
    // (or a logger) can show real-time DSP load.
    static constexpr size_t TICK_LATENCY_WINDOW = 100;
    std::array<float, TICK_LATENCY_WINDOW> tick_latencies_{};
    size_t                                 tick_latency_pos_ = 0;

    // ---- Per-stream test-tone phase (so each stream gets a stable tone) ----
    std::array<float, MAX_STREAMS> stream_tone_phase_{};

    // ---- Throttle for diagnostic feeds ----
    // Engine ticks at ~100 Hz; copying vector-of-float across threads at
    // that rate floods the GUI event queue (the long-hang/short-update
    // pattern). Throttling these to ~20 Hz keeps the visualization useful
    // while the GUI thread stays responsive.
    int diag_tick_counter_ = 0;

    // ---- HW-audio auto-recovery ----
    // Periodically re-init the always-on monitor / mic-capture devices if
    // the OS dropped them (USB unplug, sleep/wake, etc.). The counter
    // ticks every processTick and the recovery check runs once it crosses
    // RECOVERY_TICK_PERIOD so we don't hammer a permanently-disconnected
    // device on every tick.
    int diag_recovery_counter_ = 0;
    static constexpr int RECOVERY_TICK_PERIOD = 300;  ///< ~3 s at 10 ms tick

    void recoverAudioDevicesIfNeeded();
};

} // namespace gw
