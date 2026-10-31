#pragma once

#include "GuiUtil.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CaCodePage; }
QT_END_NAMESPACE

namespace gui {

class CaCodePage : public PageBase
{
    Q_OBJECT
public:
    explicit CaCodePage(QWidget* parent = nullptr);
    ~CaCodePage() override;
    void refresh() override;
    int stage() const override { return 0; }
    void clearContents() override {}    // the codes do not come from a recording

private slots:
    void updatePlots();

private:
    void buildTable();
    Ui::CaCodePage* ui;
    bool tableBuilt_ = false;
};

} // namespace gui
