#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class RawSignalPage; }
QT_END_NAMESPACE

namespace gui {

class RawSignalPage : public PageBase
{
    Q_OBJECT
public:
    explicit RawSignalPage(QWidget* parent = nullptr);
    ~RawSignalPage() override;
    void refresh() override;
    int stage() const override { return 0; }
    void clearContents() override;

protected:
    void showEvent(QShowEvent* e) override;

private slots:
    void analyse();
    void analyseSpectrogram();

private:
    Ui::RawSignalPage* ui;
    bool done_ = false;
    QString lastFile_;
};

} // namespace gui
