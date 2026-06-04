/**
 * @file help_dialog.hpp
 * @brief Multi-topic help dialog with tabbed sections covering user
 *        guidance, RF/DSP concepts, keyboard shortcuts, troubleshooting,
 *        and credits/about.
 *
 * Reachable from the Help menu. Each tab is a scrollable rich-text
 * QTextBrowser so links work for jumping between sections.
 */
#pragma once

#include <QDialog>

class QTabWidget;
class QTextBrowser;

namespace dsca {

class HelpDialog : public QDialog {
    Q_OBJECT
public:
    explicit HelpDialog(QWidget* parent = nullptr);

    /** Show a specific tab when opening (0 = Guide, 1 = Concepts, …) */
    void showTab(int index);

private:
    void buildUi();
    QTextBrowser* makeBrowser(const QString& html);

    QString guideHtml() const;
    QString conceptsHtml() const;
    QString shortcutsHtml() const;
    QString troubleshootingHtml() const;
    QString aboutHtml() const;

    QTabWidget* tabs_ = nullptr;
};

} // namespace dsca
