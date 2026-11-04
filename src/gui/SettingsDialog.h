#pragma once

#include "core/Settings.h"

#include <QDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class SettingsDialog; }
QT_END_NAMESPACE

namespace gui {

class SettingsDialog : public QDialog
{
    Q_OBJECT
public:
    explicit SettingsDialog(const gnss::ReceiverSettings& s, QWidget* parent = nullptr);
    ~SettingsDialog() override;
    gnss::ReceiverSettings settings() const;

private:
    void load(const gnss::ReceiverSettings& s);
    void updateDerived();
    void openSourceGroup();

    Ui::SettingsDialog* ui;
    gnss::ReceiverSettings base_;
};

} // namespace gui
