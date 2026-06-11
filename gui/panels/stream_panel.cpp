/**
 * @file stream_panel.cpp
 */
#include "stream_panel.hpp"
#include "../audio_engine.hpp"
#include "../style.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QDialog>
#include <QPushButton>
#include <cstdio>

namespace gw {

StreamPanel::StreamPanel(AppState& state, QWidget* parent)
    : QWidget(parent), state_(state)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING,
                              style::dim::PANEL_PADDING);
    root->setSpacing(style::dim::ITEM_SPACING);

    auto* title = new QLabel("MULTI-STREAM AUDIO");
    title->setObjectName("sectionTitle");
    root->addWidget(title);

    // Minimum content width sized for the 10-column grid. The dock's
    // scroll area handles overflow on narrow windows; we don't try to
    // compress columns below their natural width because that produces
    // overlapping labels and unreadable spinboxes.
    setMinimumWidth(840);

    auto* grid = new QGridLayout();
    // Column stretch + minimum widths. Stretch governs how extra space
    // is distributed; minimums prevent each column from collapsing
    // below the natural width of its widget (which causes overlap).
    grid->setColumnStretch(0, 0);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(2, 1);
    grid->setColumnStretch(3, 1);
    grid->setColumnStretch(4, 1);
    grid->setColumnStretch(5, 1);  // mode (Opus application)
    grid->setColumnStretch(6, 1);  // source
    grid->setColumnStretch(7, 1);  // tone Hz / file button / mic device
    grid->setColumnStretch(8, 0);  // record button
    grid->setColumnStretch(9, 2);  // level
    grid->setColumnMinimumWidth(0, 24);   // enable checkbox
    grid->setColumnMinimumWidth(1, 80);   // stream name
    grid->setColumnMinimumWidth(2, 80);   // kbps spinbox
    grid->setColumnMinimumWidth(3, 70);   // weight spinbox
    grid->setColumnMinimumWidth(4, 76);   // channels combo
    grid->setColumnMinimumWidth(5, 90);   // mode combo
    grid->setColumnMinimumWidth(6, 100);  // source combo
    grid->setColumnMinimumWidth(7, 96);   // tone Hz / file / mic
    grid->setColumnMinimumWidth(8, 28);   // rec button
    grid->setColumnMinimumWidth(9, 100);  // level meter
    grid->setHorizontalSpacing(style::dim::ITEM_SPACING);
    grid->setVerticalSpacing(4);

    grid->addWidget(new QLabel("On"),         0, 0);
    grid->addWidget(new QLabel("Stream"),     0, 1);
    grid->addWidget(new QLabel("kbps"),       0, 2);
    grid->addWidget(new QLabel("Weight"),     0, 3);
    grid->addWidget(new QLabel("Channels"),   0, 4);
    grid->addWidget(new QLabel("Mode"),       0, 5);
    grid->addWidget(new QLabel("Source"),     0, 6);
    grid->addWidget(new QLabel("Tone / File"),0, 7);
    grid->addWidget(new QLabel("Rec"),        0, 8);
    grid->addWidget(new QLabel("Level"),      0, 9);

    for (int i = 0; i < static_cast<int>(MAX_STREAMS); ++i) {
        Row& r = rows_[static_cast<size_t>(i)];

        r.enable = new QCheckBox();
        r.enable->setToolTip(
            "Enable this stream. When enabled it consumes a share of the "
            "total bitrate budget (proportional to its Weight) and its "
            "decoded audio routes to its assigned output channel.");
        r.name   = new QLabel(QString("Stream %1").arg(i));
        r.bitrate = new QSpinBox();
        r.bitrate->setRange(6, 510);
        r.bitrate->setSuffix(" kbps");
        r.bitrate->setToolTip(
            "Opus encoder target bitrate (kbps). Valid Opus range is "
            "6–510. Low end is voice-only; ~32 kbps is music at moderate "
            "quality; 64–128 kbps is good music quality. Effective bitrate "
            "may be lower if the channel budget can't fit all enabled "
            "streams (the coordinator allocates proportionally to Weight).");
        r.weight  = new QDoubleSpinBox();
        r.weight->setRange(0.1, 10.0);
        r.weight->setSingleStep(0.1);
        r.weight->setDecimals(1);
        r.weight->setToolTip(
            "Bandwidth-share weight relative to other enabled streams. "
            "Stream gets share = weight / sum(enabled weights). A 2.0 "
            "weight stream takes twice the bitrate of a 1.0 stream.");
        r.channels = new QComboBox();
        r.channels->addItem("Mono",   1);
        r.channels->addItem("Stereo", 2);
        r.channels->setToolTip(
            "Opus encoder channel count. Stereo uses internal joint-stereo "
            "coding (M/S inside Opus) at modest extra cost — typically "
            "+30–50% bitrate for a noticeable stereo image.");
        r.mode = new QComboBox();
        r.mode->addItem("Audio",    static_cast<int>(OpusApplication::Audio));
        r.mode->addItem("VoIP",     static_cast<int>(OpusApplication::VoIP));
        r.mode->addItem("LowDelay", static_cast<int>(OpusApplication::LowDelay));
        r.mode->setToolTip(
            "Opus application mode. 'Audio' (default) is best for music "
            "and mixed content. 'VoIP' tunes SILK aggressively for "
            "speech-only streams at sub-32 kbps — choose for talk-radio "
            "feeds. 'LowDelay' disables SILK (CELT-only) cutting ~5 ms "
            "of algorithmic delay; worse for low-bitrate speech, useful "
            "for live broadcast monitoring.");
        r.source = new QComboBox();
        r.source->addItem("Test Tone", static_cast<int>(StreamAudioSource::TestTone));
        r.source->addItem("Silence",   static_cast<int>(StreamAudioSource::Silence));
        r.source->addItem("Mic",       static_cast<int>(StreamAudioSource::Microphone));
        // File (looped WAV) is a fully-supported source (engine + the file
        // picker below) but was missing here, so a File-source stream (e.g.
        // loaded from a config) was unselectable and refreshFromState's
        // setCurrentIndex(3) silently no-op'd on a 3-item combo.
        r.source->addItem("File",      static_cast<int>(StreamAudioSource::File));
        r.source->setToolTip(
            "Audio source: Test Tone (programmable sine), Silence (digital "
            "zero for noise-floor measurement), Mic (system default capture "
            "device), File (looped WAV — click the … button to pick).");
        r.tone_hz = new QSpinBox();
        r.tone_hz->setRange(50, 8000);
        r.tone_hz->setValue(440 + i * 110);   // distinct per stream
        r.tone_hz->setSuffix(" Hz");
        r.tone_hz->setToolTip(
            "Test tone frequency. Each stream defaults to a different "
            "frequency so streams are audibly distinguishable on the "
            "decoded side.");
        r.pick_file = new QToolButton();
        r.pick_file->setText("…");
        r.pick_file->setToolTip("Pick WAV file (used when Source = File)");
        r.pick_file->setVisible(false);  // toggled by source change

        // ◉ button: pick a specific capture device for this stream's
        // microphone source. Each enabled mic stream can target a
        // different physical input. Default (system default) shown when
        // input_device == -1.
        r.pick_input = new QToolButton();
        r.pick_input->setText("◉");
        r.pick_input->setToolTip(
            "Pick the capture device for this stream's microphone.\n"
            "Each mic-source stream can use a different physical input —\n"
            "useful for routing two mics through two streams.");
        r.pick_input->setVisible(false);  // shown only when Source = Mic
        r.record_btn = new QToolButton();
        r.record_btn->setText("●");
        r.record_btn->setCheckable(true);
        r.record_btn->setToolTip("Record decoded stream audio to WAV");
        r.record_btn->setStyleSheet(
            "QToolButton { color:#8E8E93; }"
            "QToolButton:checked { color:#FF453A; }"
        );
        r.level = new QProgressBar();
        r.level->setRange(-60, 0);
        r.level->setFormat("%v dBFS");
        r.level->setValue(-60);

        const int row = i + 1;
        grid->addWidget(r.enable,   row, 0);
        grid->addWidget(r.name,     row, 1);
        grid->addWidget(r.bitrate,  row, 2);
        grid->addWidget(r.weight,   row, 3);
        grid->addWidget(r.channels, row, 4);
        grid->addWidget(r.mode,     row, 5);
        grid->addWidget(r.source,   row, 6);
        // Column 7 shares: tone_hz spinbox (when Source=TestTone), the
        // file-picker button (when Source=File), and the input-device
        // picker (when Source=Mic). The Source change handler toggles
        // visibility so only the relevant control appears.
        {
            auto* shared = new QWidget;
            auto* h = new QHBoxLayout(shared);
            h->setContentsMargins(0,0,0,0); h->setSpacing(2);
            h->addWidget(r.tone_hz, 1);
            h->addWidget(r.pick_file);
            h->addWidget(r.pick_input);
            grid->addWidget(shared, row, 7);
        }
        grid->addWidget(r.record_btn, row, 8);
        grid->addWidget(r.level,      row, 9);

        const int id = i;
        connect(r.enable, &QCheckBox::toggled, this,
                [this, id](bool on){ onEnableToggled(id, on); });
        connect(r.bitrate,
                static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                this, [this, id](int v){ onBitrateChanged(id, v); });
        connect(r.weight,
                static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                this, [this, id](double v){ onWeightChanged(id, v); });
        connect(r.channels,
                static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                this, [this, id](int idx){ onChannelsChanged(id, idx); });
        connect(r.mode,
                static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                this, [this, id](int idx){
                    if (updating_) return;
                    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
                    {
                        std::lock_guard<std::mutex> lk(state_.mtx);
                        auto v = static_cast<OpusApplication>(idx);
                        state_.stream_configs[static_cast<size_t>(id)].app = v;
                    }
                    emit streamConfigChanged();
                });
        connect(r.source,
                static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                this, [this, id](int idx){ onSourceChanged(id, idx); });
        connect(r.tone_hz,
                static_cast<void(QSpinBox::*)(int)>(&QSpinBox::valueChanged),
                this, [this, id](int hz){ onToneFreqChanged(id, hz); });
        connect(r.pick_file, &QToolButton::clicked, this,
                [this, id]{ onPickFile(id); });
        connect(r.pick_input, &QToolButton::clicked, this,
                [this, id]{ onPickInputDevice(id); });
        connect(r.record_btn, &QToolButton::toggled, this,
                [this, id](bool){ onToggleRecord(id); });
    }
    root->addLayout(grid);
    root->addStretch();
    refreshFromState();
}

void StreamPanel::refreshFromState() {
    updating_ = true;
    std::lock_guard<std::mutex> lk(state_.mtx);
    for (size_t i = 0; i < MAX_STREAMS; ++i) {
        const auto& sc = state_.stream_configs[i];
        Row& r = rows_[i];
        r.enable->setChecked(sc.enabled);
        r.bitrate->setValue(static_cast<int>(sc.bitrate_bps / 1000));
        r.weight->setValue(static_cast<double>(sc.weight));
        r.channels->setCurrentIndex(sc.channels == 2 ? 1 : 0);
        r.mode->setCurrentIndex(static_cast<int>(sc.app));
        r.source->setCurrentIndex(static_cast<int>(sc.source));
        r.tone_hz->setValue(static_cast<int>(sc.tone_freq_hz));
        // Toggle the shared column-7 widget per source: tone Hz for
        // TestTone, file picker for File, device picker for Mic.
        r.tone_hz->setVisible(sc.source == StreamAudioSource::TestTone);
        r.pick_file->setVisible(sc.source == StreamAudioSource::File);
        r.pick_input->setVisible(sc.source == StreamAudioSource::Microphone);
        r.name->setText(QString::fromUtf8(sc.name));
    }
    updating_ = false;
}

void StreamPanel::onStreamLevels(const std::array<float, MAX_STREAMS>& rms_db) {
    for (size_t i = 0; i < MAX_STREAMS; ++i) {
        int v = static_cast<int>(rms_db[i]);
        if (v < -60) v = -60;
        if (v > 0)   v = 0;
        rows_[i].level->setValue(v);
    }
}

void StreamPanel::onEnableToggled(int id, bool on) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].enabled = on;
    }
    emit streamConfigChanged();
}

void StreamPanel::onBitrateChanged(int id, int kbps) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].bitrate_bps =
            static_cast<uint32_t>(kbps) * 1000u;
    }
    emit streamConfigChanged();
}

void StreamPanel::onWeightChanged(int id, double w) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].weight =
            static_cast<float>(w);
    }
    emit streamConfigChanged();
}

void StreamPanel::onChannelsChanged(int id, int idx) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].channels =
            (idx == 1) ? 2 : 1;
    }
    emit streamConfigChanged();
}

void StreamPanel::onSourceChanged(int id, int idx) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    StreamAudioSource src = static_cast<StreamAudioSource>(idx);
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].source = src;
    }
    auto& r = rows_[static_cast<size_t>(id)];
    r.tone_hz->setVisible(src == StreamAudioSource::TestTone);
    r.pick_file->setVisible(src == StreamAudioSource::File);
    r.pick_input->setVisible(src == StreamAudioSource::Microphone);
    emit streamConfigChanged();
}

void StreamPanel::onPickFile(int id) {
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    QString path = QFileDialog::getOpenFileName(this,
        "Pick WAV file for stream", QString(),
        "WAV files (*.wav);;All files (*)");
    if (path.isEmpty()) return;
    QByteArray utf8 = path.toUtf8();
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        auto& sc = state_.stream_configs[static_cast<size_t>(id)];
        std::strncpy(sc.file_path, utf8.constData(), sizeof(sc.file_path) - 1);
        sc.file_path[sizeof(sc.file_path) - 1] = '\0';
    }
    rows_[static_cast<size_t>(id)].pick_file->setToolTip(path);
    emit streamConfigChanged();
}

void StreamPanel::onPickInputDevice(int id) {
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    if (!engine_) return;

    // Build a small picker dialog: list of capture devices + a default
    // entry. Selection writes back to state_.stream_configs[id].input_device.
    auto devices = engine_->enumerateCaptureDevices();

    QDialog dlg(this);
    dlg.setWindowTitle(QString("Input device — Stream %1").arg(id));
    dlg.setMinimumWidth(360);
    auto* lay = new QVBoxLayout(&dlg);
    auto* heading = new QLabel(
        "<b>Pick capture device for this stream</b><br/>"
        "<span style=\"color:#8E8E93;\">"
        "Two streams using the same device share one capture session."
        "</span>");
    heading->setTextFormat(Qt::RichText);
    heading->setWordWrap(true);
    lay->addWidget(heading);

    auto* combo = new QComboBox();
    combo->addItem("System Default", -1);
    int current_idx = 0;
    int target_dev = -1;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        target_dev = state_.stream_configs[static_cast<size_t>(id)].input_device;
    }
    for (size_t i = 0; i < devices.size(); ++i) {
        QString label = QString::fromUtf8(devices[i].name.c_str());
        if (devices[i].is_default) label += "  (default)";
        combo->addItem(label, static_cast<int>(i));
        if (static_cast<int>(i) == target_dev) {
            current_idx = combo->count() - 1;
        }
    }
    if (target_dev == -1) current_idx = 0;
    combo->setCurrentIndex(current_idx);
    lay->addWidget(combo);

    auto* btn_row = new QHBoxLayout();
    btn_row->addStretch();
    auto* cancel = new QPushButton("Cancel");
    auto* ok     = new QPushButton("Apply");
    ok->setDefault(true);
    btn_row->addWidget(cancel);
    btn_row->addWidget(ok);
    lay->addLayout(btn_row);
    connect(cancel, &QPushButton::clicked, &dlg, &QDialog::reject);
    connect(ok,     &QPushButton::clicked, &dlg, &QDialog::accept);

    if (dlg.exec() != QDialog::Accepted) return;

    int chosen = combo->currentData().toInt();
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].input_device = chosen;
    }
    QString tip = (chosen == -1) ? QString("System default")
                                 : combo->currentText();
    rows_[static_cast<size_t>(id)].pick_input->setToolTip(
        QString("Input device: %1\n\nClick to change.").arg(tip));
    emit streamConfigChanged();
}

void StreamPanel::onToggleRecord(int id) {
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    if (!engine_) return;
    auto& btn = rows_[static_cast<size_t>(id)].record_btn;
    bool want_record = btn->isChecked();
    if (want_record) {
        QString path = QFileDialog::getSaveFileName(this,
            QString("Record stream %1 audio").arg(id), QString(),
            "WAV files (*.wav);;All files (*)");
        if (path.isEmpty()) {
            btn->setChecked(false);
            return;
        }
        if (!engine_->startStreamRecording(static_cast<size_t>(id),
                                            path.toStdString())) {
            btn->setChecked(false);
        }
    } else {
        engine_->stopStreamRecording(static_cast<size_t>(id));
    }
}

void StreamPanel::onToneFreqChanged(int id, int hz) {
    if (updating_) return;
    if (id < 0 || id >= static_cast<int>(MAX_STREAMS)) return;
    {
        std::lock_guard<std::mutex> lk(state_.mtx);
        state_.stream_configs[static_cast<size_t>(id)].tone_freq_hz =
            static_cast<float>(hz);
    }
    emit streamConfigChanged();
}

} // namespace gw
