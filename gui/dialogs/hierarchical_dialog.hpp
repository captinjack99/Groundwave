/**
 * @file hierarchical_dialog.hpp
 * @brief Configuration dialog for hierarchical-modulation parameters.
 *
 * Exposes:
 *   - Mode dropdown (None / 16 preset combinations / Custom)
 *   - Alpha slider 1.0–4.0 (constellation HP-vs-LP energy split)
 *   - Custom HP-bits spinner (when Mode = Custom)
 *
 * Edits propagate to AudioEngineConfig.hier via the parent main window.
 */
#pragma once

#include "../../include/hierarchical_mod.hpp"
#include "../audio_engine.hpp"
#include "../widgets/hier_flow_widget.hpp"
#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLabel>
#include <QCheckBox>

namespace dsca {

class HierarchicalDialog : public QDialog {
    Q_OBJECT
public:
    /** @param cfg  Current AudioEngineConfig; on accept(), `engineConfig()`
     *              returns the modified copy. */
    explicit HierarchicalDialog(const AudioEngineConfig& cfg,
                                 QWidget* parent = nullptr);

    /** Returns the modified engine config (only meaningful after accept). */
    const AudioEngineConfig& engineConfig() const { return cfg_; }

private slots:
    void onModeChanged(int idx);
    void onAlphaChanged(double v);
    void onEnabledToggled(bool on);
    void onCustomBaseChanged(int idx);
    void onCustomHpChanged(int v);
    void onApply();

private:
    void buildUi();
    void refreshLabels();

    AudioEngineConfig cfg_;

    QCheckBox*      enable_cb_;
    QComboBox*      mode_combo_;
    QDoubleSpinBox* alpha_spin_;
    QComboBox*      custom_base_combo_;   // Custom: base modulation
    QSpinBox*       custom_hp_spin_;      // Custom: HP bits
    QLabel*         info_label_;          // shows resolved HP/LP/total
    HierFlowWidget* flow_widget_;         // visual signal-flow diagram
};

} // namespace dsca
