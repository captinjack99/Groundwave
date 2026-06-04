/**
 * @file help_dialog.cpp
 */
#include "help_dialog.hpp"
#include "../style.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QTextBrowser>
#include <QPushButton>
#include <QLabel>

namespace dsca {

HelpDialog::HelpDialog(QWidget* parent) : QDialog(parent) {
    setWindowTitle("DSCA-NG — Help");
    setMinimumSize(820, 620);
    buildUi();
}

void HelpDialog::showTab(int index) {
    if (tabs_ && index >= 0 && index < tabs_->count()) {
        tabs_->setCurrentIndex(index);
    }
}

void HelpDialog::buildUi() {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(20, 18, 20, 16);
    root->setSpacing(14);

    auto* title = new QLabel("DSCA-NG Help");
    title->setStyleSheet(
        "font-size: 22px; font-weight: 300; color: #F2F2F7; "
        "letter-spacing: 0.5px;");
    root->addWidget(title);

    tabs_ = new QTabWidget(this);
    tabs_->setDocumentMode(true);
    tabs_->addTab(makeBrowser(guideHtml()),           "User Guide");
    tabs_->addTab(makeBrowser(conceptsHtml()),        "RF / DSP Concepts");
    tabs_->addTab(makeBrowser(shortcutsHtml()),       "Keyboard Shortcuts");
    tabs_->addTab(makeBrowser(troubleshootingHtml()), "Troubleshooting");
    tabs_->addTab(makeBrowser(aboutHtml()),           "About");
    root->addWidget(tabs_, 1);

    auto* button_row = new QHBoxLayout;
    button_row->addStretch();
    auto* close_btn = new QPushButton("Close");
    close_btn->setFixedWidth(90);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::close);
    button_row->addWidget(close_btn);
    root->addLayout(button_row);
}

QTextBrowser* HelpDialog::makeBrowser(const QString& html) {
    auto* tb = new QTextBrowser();
    tb->setOpenExternalLinks(true);
    tb->setStyleSheet(
        "QTextBrowser { background: #0E0E14; border: 1px solid #1C1C24; "
        "  border-radius: 8px; padding: 14px; color: #C7C7CC; "
        "  font-size: 12px; line-height: 1.55; }");
    tb->setHtml(html);
    return tb;
}

// =========================================================================
// Content
// =========================================================================

QString HelpDialog::guideHtml() const {
    return R"HTML(
<h2 style="color:#F2F2F7; font-weight:300; margin-bottom:0;">User Guide</h2>
<p style="color:#8E8E93;">A 5-minute orientation. Skim the headings; read what you need.</p>

<h3 style="color:#0099FF;">1. The first time you run DSCA-NG</h3>
<p>The application opens in <b>software loopback</b> mode by default. The
transmitter encodes audio into an OFDM signal; the receiver decodes the
same signal back to audio — entirely inside the software, with no
soundcard or radio involved. This lets you see every part of the chain
working before connecting real hardware.</p>
<p>Press <b>F5</b> (or click <code>TX</code> in the TX panel) to enable
transmission. The waterfall fills with a colored signal, the
constellation widget populates, and the InfoPanel shows live bitrate.</p>

<h3 style="color:#0099FF;">2. Picking a Preset</h3>
<p>The <b>F1</b>–<b>F8</b> keys load the 8 built-in presets. Each preset
chooses a complete modem configuration (modulation, FEC, FFT, sample
rate, center frequency):</p>
<ul>
  <li><b>F1 — Robust:</b> BPSK / Rate 1/4 — survives noisy channels but
      carries little audio</li>
  <li><b>F2 — Standard:</b> QPSK / Rate 1/2 — balanced default</li>
  <li><b>F3 — HD Audio:</b> 16-QAM / Rate 3/4 — clean stereo at moderate SNR</li>
  <li><b>F4 — High Capacity:</b> 64-QAM / Rate 4/5 — needs good SNR</li>
  <li><b>F5 — Ultra HD:</b> 256-QAM / Rate 8/9 — wide channel, premium audio</li>
  <li><b>F6 — Broadcast Studio:</b> 1024-QAM, 192 kHz SR — top quality, top SNR</li>
  <li><b>F7 — Emergency:</b> BPSK, 1/4, tiny FFT — last-resort robustness</li>
  <li><b>F8 — Custom:</b> empty slot, save your own with File → Save Preset</li>
</ul>

<h3 style="color:#0099FF;">3. Reading the Status</h3>
<p>The <b>Status</b> dock on the right shows live RX measurements:</p>
<ul>
  <li><b>SNR / EVM / BER</b> — signal quality from best to most
      revealing of trouble.</li>
  <li><b>Active / Data / Pilot subcarriers</b> — derived from your
      OFDM configuration.</li>
  <li><b>Spectral Efficiency</b> — bits per second per hertz of
      occupied bandwidth.</li>
  <li><b>Hierarchical Layers</b> — only shown when hierarchical
      modulation is on; see Concepts.</li>
</ul>

<h3 style="color:#0099FF;">4. Adding Multi-Stream Audio</h3>
<p>Open the <b>Streams + Scope</b> dock at the bottom. Each row
configures one of 8 independent audio streams. Enable a stream, set its
bitrate, pick a source (Test Tone / Silence / Mic / File), and the
engine starts carrying that stream over the modem.</p>
<p>Each enabled stream's decoded audio is routed to its own output
channel of the playback device. On a stereo speaker (2 channels),
streams 0–7 wrap modulo-2 into L/R with soft clipping. On a
multi-channel virtual cable (VB-CABLE 16ch, VoiceMeeter), you get
true 1-to-1 routing.</p>

<h3 style="color:#0099FF;">5. Going Live: Hardware Audio</h3>
<p>Open <b>Settings → Audio Devices…</b>. Pick a Playback device (where
the modulated TX signal goes — typically a virtual audio cable feeding
your FM exciter) and a Capture device (where the RX signal comes
back). Tick <b>Enable Hardware Audio</b> and click <b>Apply</b>.</p>
<p>The TX/RX path now traverses real audio hardware; loopback is
disabled. The InfoPanel still shows live status; the SNR readout
reflects the actual channel quality.</p>

<h3 style="color:#0099FF;">6. Persistence</h3>
<p><b>File → Save Config…</b> writes the entire app state (presets,
streams, alarm thresholds, current modcod, hierarchical settings) to a
JSON file. <b>Load Config…</b> restores it. Use this to keep
per-installation configurations or to share a setup with a remote site.</p>
)HTML";
}

QString HelpDialog::conceptsHtml() const {
    return R"HTML(
<h2 style="color:#F2F2F7; font-weight:300; margin-bottom:0;">RF / DSP Concepts</h2>
<p style="color:#8E8E93;">What every term in the GUI means, in plain
language.</p>

<h3 style="color:#0099FF;">OFDM</h3>
<p><b>Orthogonal Frequency-Division Multiplexing.</b> Splits the
available bandwidth into many narrow subcarriers, each carrying a small
piece of the data. Tolerates multipath much better than a single wide
carrier. <i>FFT size</i> is the number of subcarriers; larger FFT means
narrower subcarriers (finer frequency resolution) but longer symbol
duration.</p>

<h3 style="color:#0099FF;">Cyclic Prefix (CP)</h3>
<p>A copy of the end of each OFDM symbol prepended to its front. Acts
as a guard interval against multipath delay spread — as long as
echoes arrive within CP duration, they don't cause inter-symbol
interference. CP fractions in this app: 1/4 (heavy multipath), 1/8
(default), 1/16, 1/32 (clean channels).</p>

<h3 style="color:#0099FF;">Modulation (Constellation)</h3>
<p>How many bits each subcarrier carries per symbol period. Higher
order = more bits but needs higher SNR:</p>
<table style="margin-left:18px; color:#C7C7CC;">
<tr><td><b>BPSK</b></td><td>1 bit/sym</td><td>~3 dB SNR</td></tr>
<tr><td><b>QPSK</b></td><td>2 bits/sym</td><td>~6 dB SNR</td></tr>
<tr><td><b>16-QAM</b></td><td>4 bits/sym</td><td>~12 dB SNR</td></tr>
<tr><td><b>64-QAM</b></td><td>6 bits/sym</td><td>~18 dB SNR</td></tr>
<tr><td><b>256-QAM</b></td><td>8 bits/sym</td><td>~24 dB SNR</td></tr>
<tr><td><b>1024-QAM</b></td><td>10 bits/sym</td><td>~30 dB SNR</td></tr>
<tr><td><b>4096-QAM</b></td><td>12 bits/sym</td><td>~36 dB SNR</td></tr>
</table>

<h3 style="color:#0099FF;">FEC (Forward Error Correction)</h3>
<p><b>LDPC</b> (Low-Density Parity-Check) codes add redundant bits so the
receiver can fix bit errors. <i>Code rate</i> = info / total bits. Lower
rate = stronger protection at the cost of throughput:</p>
<ul>
  <li><b>Rate 1/4</b> — strongest, suitable for very weak channels</li>
  <li><b>Rate 1/2</b> — balanced default</li>
  <li><b>Rate 9/10</b> — near-Shannon-limit, needs clean channel</li>
</ul>
<p>An <b>outer Reed-Solomon</b> code (16 parity bytes per block) wraps
the LDPC info field to mop up residual byte errors in the LDPC
waterfall region. Default on.</p>

<h3 style="color:#0099FF;">Hierarchical Modulation</h3>
<p>A single QAM constellation that encodes <b>two</b> bit streams with
different protection:</p>
<ul>
  <li>The <b>HP (High Priority)</b> layer carries the most important
      bits, robust to noise.</li>
  <li>The <b>LP (Low Priority)</b> layer carries enhancement bits,
      requires higher SNR.</li>
</ul>
<p>The constellation parameter <b>α</b> controls the HP/LP energy split.
α = 1 means uniform (no hierarchy); α = 2 is DVB-T's default with
roughly 6 dB HP/LP threshold gap; α = 4 gives maximum HP protection.</p>
<p>This app pairs HP/LP with <b>Mid/Side</b> stereo:</p>
<ul>
  <li><b>Mid</b> (stream 0) rides HP — survives even at marginal SNR.</li>
  <li><b>Side</b> (stream 1) rides LP — drops out first under stress.</li>
</ul>
<p>When LP is failing, the receiver synthesizes a plausible Side from
the decoded Mid (all-pass-cascade decorrelator with smooth
LLR-confidence-driven crossfade), so the stereo image degrades
gracefully to a coherent decorrelated-stereo before going mono.</p>

<h3 style="color:#0099FF;">AMC and VCM</h3>
<p><b>Adaptive Modulation and Coding</b> (AMC): the receiver measures
SNR and the transmitter chooses the most-throughput ModCod that
still decodes reliably. <b>Variable Coding and Modulation</b> (VCM): a
fixed schedule that cycles through multiple ModCods within a
superframe, useful for broadcasts that need both robust and
high-throughput slots in the same channel.</p>

<h3 style="color:#0099FF;">PAPR Reduction</h3>
<p>OFDM signals have <b>P</b>eak-to-<b>A</b>verage <b>P</b>ower
<b>R</b>atio of 8–13 dB. High PAPR drives FM exciters or RF
amplifiers into nonlinear behavior, creating out-of-band emissions.
<i>Tone Reservation</i> reserves a few guard subcarriers and fills
them with kernel-shifted tones that null large time-domain peaks.</p>

<h3 style="color:#0099FF;">AFC, AGC, Squelch</h3>
<ul>
  <li><b>AFC</b> (Automatic Frequency Correction): a 2nd-order PLL
      tracks small residual carrier offset; an SRO estimator tracks
      sample-clock drift in PPM.</li>
  <li><b>AGC</b> (Automatic Gain Control): scales RX input so its RMS
      sits at a target level. <i>Attack</i> = how fast the gain
      responds to peaks; <i>Release</i> = how fast it recovers.</li>
  <li><b>Squelch</b>: energy-detector that mutes RX during silence,
      with hysteresis so it doesn't chatter on marginal signals.</li>
</ul>

<h3 style="color:#0099FF;">Sync FSM</h3>
<p>The receiver progresses through <b>Searching</b> →
<b>Acquiring</b> → <b>Locked</b> → <b>Tracking</b> → <b>Lost</b>
based on Zadoff-Chu preamble correlation, pilot quality, and PLS
decode confirmations. The status bar shows the current state.</p>
)HTML";
}

QString HelpDialog::shortcutsHtml() const {
    return R"HTML(
<h2 style="color:#F2F2F7; font-weight:300; margin-bottom:0;">Keyboard Shortcuts</h2>
<p style="color:#8E8E93;">All single-key and Ctrl-combination shortcuts.</p>

<h3 style="color:#0099FF;">Preset Selection</h3>
<table cellpadding="6">
<tr><td><code style="color:#0099FF;">F2</code></td><td>Standard (QPSK, 1/2)</td></tr>
<tr><td><code style="color:#0099FF;">F3</code></td><td>HD Audio (16-QAM, 3/4)</td></tr>
<tr><td><code style="color:#0099FF;">F4</code></td><td>High Capacity (64-QAM, 4/5)</td></tr>
<tr><td><code style="color:#0099FF;">F6</code></td><td>Broadcast Studio (1024-QAM, 9/10)</td></tr>
<tr><td><code style="color:#0099FF;">F7</code></td><td>Emergency (BPSK, 1/4)</td></tr>
<tr><td><code style="color:#0099FF;">F8</code></td><td>Custom (user-defined; empty by default)</td></tr>
</table>
<p style="color:#8E8E93;">The <b>Robust</b> and <b>Ultra HD</b> presets are on the
<b>Presets</b> menu — F1 (Help) and F5 (TX) reserve those keys.</p>

<h3 style="color:#0099FF;">Transmit / Receive Control</h3>
<table cellpadding="6">
<tr><td><code style="color:#0099FF;">F5</code></td><td>Toggle TX on/off</td></tr>
<tr><td><code style="color:#0099FF;">F9</code></td><td>Start engine</td></tr>
<tr><td><code style="color:#0099FF;">F10</code></td><td>Stop engine</td></tr>
</table>

<h3 style="color:#0099FF;">File / Export</h3>
<table cellpadding="6">
<tr><td><code style="color:#0099FF;">Ctrl+S</code></td><td>Save configuration to JSON</td></tr>
<tr><td><code style="color:#0099FF;">Ctrl+O</code></td><td>Load configuration from JSON</td></tr>
<tr><td><code style="color:#0099FF;">Ctrl+E</code></td><td>Export spectrum to PNG</td></tr>
</table>

<h3 style="color:#0099FF;">Help</h3>
<table cellpadding="6">
<tr><td><code style="color:#0099FF;">F1</code></td><td>Open this Help dialog</td></tr>
</table>
<p style="color:#8E8E93;">Theme (Light / Dark) and panel layout are on the
<b>View</b> menu.</p>

<h3 style="color:#0099FF;">Spectrum Widget</h3>
<table cellpadding="6">
<tr><td>Mouse drag</td><td>Pan frequency axis (when zoomed)</td></tr>
<tr><td>Mouse wheel</td><td>Zoom frequency axis</td></tr>
<tr><td>Click</td><td>Place primary cursor (frequency readout)</td></tr>
<tr><td>Shift + click</td><td>Place delta cursor (Δf readout)</td></tr>
<tr><td>Right click</td><td>Clear cursors</td></tr>
</table>

<h3 style="color:#0099FF;">Constellation Widget</h3>
<table cellpadding="6">
<tr><td>Click</td><td>Toggle EVM-detail mode (per-symbol error magnitude)</td></tr>
</table>
)HTML";
}

QString HelpDialog::troubleshootingHtml() const {
    return R"HTML(
<h2 style="color:#F2F2F7; font-weight:300; margin-bottom:0;">Troubleshooting</h2>
<p style="color:#8E8E93;">Common problems and the fix for each.</p>

<h3 style="color:#0099FF;">"No SNR / Sync Lost" right after enabling TX</h3>
<p>Wait 1–2 seconds. In software loopback the receiver needs a Zadoff-Chu
preamble to lock; preambles are sent every 50 frames by default. The
SyncFSM transitions Searching → Acquiring → Locked over a few hundred
milliseconds.</p>
<p>If it persists: open Tuning panel and check <b>AFC enabled</b>. If
you're in HW audio mode, verify the playback and capture devices
actually carry signal — try setting both to the same loopback device.</p>

<h3 style="color:#0099FF;">"Hardware audio failed to start"</h3>
<p>Two common causes:</p>
<ul>
  <li>The chosen device is exclusively held by another application.
      Close everything else using audio and retry.</li>
  <li>The device doesn't support mono. The app falls back to (1,1),
      (2,1), (1,2), (2,2), and (0,0) automatically — but a virtual
      cable with hard channel requirements may still refuse. Try a
      different device.</li>
</ul>

<h3 style="color:#0099FF;">"Slow tick" messages in the log</h3>
<p>The engine logs <code>[engine] slow tick: TX=X LB=Y RX=Z total=N ms</code>
whenever a tick exceeds 25 ms. If you're seeing this constantly with
QAM-256 or higher at FFT ≥ 1024:</p>
<ul>
  <li>Verify the <b>PWL LLR Demapper</b> is enabled (Engine menu).
      Default is on.</li>
  <li>Drop FFT size to 512 if you don't need the spectral efficiency.</li>
  <li>Lower the maximum LDPC iterations (currently 25 with the
      layered schedule).</li>
</ul>

<h3 style="color:#0099FF;">Audio sounds distorted / glitchy</h3>
<p>Check the TX meter for clipping (red bar at top). Reduce TX gain.</p>
<p>Open the Tuning panel and look at <b>AGC Ripple</b>. If &gt; 6 dB
the AGC is pumping — increase Release time, decrease Attack, or both.</p>
<p>If only one channel is glitchy and you're in M/S hierarchical mode:
that's the LP (Side) layer dropping out under marginal SNR. The
graceful-degradation algorithm crossfades to a synthesized Side, so
glitches should not be audible — if they are, file a bug.</p>

<h3 style="color:#0099FF;">"Center freq must be &lt; Nyquist"</h3>
<p>Your Fc + BW/2 exceeds SR/2 (Nyquist). Open the TX panel and either
lower Fc, narrow BW, or raise sample rate. <b>Auto-clamp to Nyquist</b>
(checkbox under the BW slider) will do this automatically going forward.</p>

<h3 style="color:#0099FF;">Hierarchical layer status shows "FAIL" but signal is fine</h3>
<p>You may have asymmetric hierarchical modulation enabled (e.g.,
QPSK/64-QAM with a 2+4 split). The receiver handles this branch but
the M/S parallel chain only activates for symmetric splits (HP bits
= LP bits, e.g., QPSK/16-QAM with 2+2 or 16-QAM/256-QAM with 4+4).
In asymmetric modes the HP/LP frame counters are derived from the
same codeword, so they show identical confidence.</p>

<h3 style="color:#0099FF;">App crashes on startup</h3>
<p>Run from a terminal: <code>dsca_ng.exe 2&gt; engine.log</code> and check
<code>engine.log</code> for the panic message. Common causes: missing
Qt6 DLLs (run <code>windeployqt</code>), or Opus library mismatch
(reinstall the matching version from vcpkg).</p>

<h3 style="color:#0099FF;">"Preset file won't load" / Config file silently ignored</h3>
<p>The config-file <code>version</code> field must match the current app
version (2). Older configs need manual conversion. Use the JSON editor
to bump the version field if the schema is compatible.</p>
)HTML";
}

QString HelpDialog::aboutHtml() const {
    return R"HTML(
<h2 style="color:#F2F2F7; font-weight:300; margin-bottom:0;">About DSCA-NG</h2>

<p style="color:#C7C7CC; font-size:13px;">
<b>DSCA-NG — Digital SCA Next Generation</b><br/>
A software-defined OFDM digital radio modem for FM SCA / soundcard-
audio digital broadcasting. Built end-to-end in C++17 with Qt 6 for
the GUI and miniaudio + Opus for I/O.</p>

<h3 style="color:#0099FF;">Features</h3>
<ul>
  <li>OFDM with FFT 64 – 16384, CP 1/4 – 1/32</li>
  <li>Modulations BPSK through QAM-4096 (uniform and hierarchical)</li>
  <li>LDPC FEC at 11 rates from 1/4 to 9/10 with layered min-sum decoding</li>
  <li>Reed-Solomon outer code (16-byte parity) for waterfall-region cleanup</li>
  <li>Up to 8 simultaneous Opus-coded audio streams with inband FEC and DTX</li>
  <li>Mid/Side stereo over hierarchical modulation with predictive Side reconstruction</li>
  <li>Adaptive Modulation &amp; Coding driven by measured SNR</li>
  <li>MMSE + Wiener channel estimation with DFT-based denoising</li>
  <li>PAPR reduction via tone reservation</li>
  <li>FCC §73.319 spectral-mask compliance via 129-tap windowed-sinc TX LPF</li>
  <li>Multi-channel audio routing (stream i → output channel i)</li>
  <li>Multi-stream + multi-channel virtual-audio-cable compatible</li>
</ul>

<h3 style="color:#0099FF;">Test infrastructure</h3>
<p>11 ctest suites covering FEC, OFDM chain, DSP primitives, end-to-end
integration, LDPC matrix correctness, Reed-Solomon error correction,
Side reconstructor, configuration persistence, and GUI smoke tests.
All build at <code>/W4 /WX /permissive-</code>.</p>

<h3 style="color:#0099FF;">License &amp; Dependencies</h3>
<p>Qt 6 (LGPL), miniaudio (public domain), libopus (BSD), FFTW 3
(optional, GPL). Codebase under the project's own license.</p>

<p style="color:#48484E; font-size:10px; margin-top:24px;">
Built %DATE% · Qt %QTVER% · MSVC %MSVCVER%
</p>
)HTML";
}

} // namespace dsca
