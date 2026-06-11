/**
 * @file preset_dialog.hpp
 */
#pragma once
#include "../../include/app_state.hpp"
#include "../style.hpp"
#include <QDialog>
#include <QTableWidget>
#include <QLineEdit>
#include <QSpinBox>

namespace gw {

class PresetDialog : public QDialog {
    Q_OBJECT
public:
    explicit PresetDialog(AppState& state, QWidget* parent = nullptr);

signals:
    void presetLoaded();

private slots:
    void onLoad();
    void onSave();
    void onRename();
    void onRowDoubleClicked(int row, int col);
    void onSelectionChanged();

private:
    void buildUi();
    void refreshTable();
    QString describePreset(const PresetConfig& p) const;

    AppState&     state_;
    QTableWidget* table_;
    QSpinBox*     slot_spin_;
    QLineEdit*    name_edit_;
};

} // namespace gw
