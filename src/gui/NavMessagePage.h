#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class NavMessagePage; }
QT_END_NAMESPACE

namespace gui {

class NavMessagePage : public PageBase
{
    Q_OBJECT
public:
    explicit NavMessagePage(QWidget* parent = nullptr);
    ~NavMessagePage() override;
    void refresh() override;
    int stage() const override { return gnss::Receiver::StageDecoded; }

public slots:
    void selectPrn(int prn);

private slots:
    void showChannel();
    void showSubframe(int index);
    void setSubframeColumns(bool galileo);
    void showInavWord(const gnss::NavChannel& n, int index);
    void showGlonassString(const gnss::NavChannel& n, int index);

private:
    Ui::NavMessagePage* ui;
};

} // namespace gui
