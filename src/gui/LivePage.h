#pragma once

#include "GuiUtil.h"
#include "core/LiveEngine.h"

#include <QMap>
#include <QVector>

QT_BEGIN_NAMESPACE
namespace Ui { class LivePage; }
QT_END_NAMESPACE

namespace gui {

// Dashboard of the running receiver: the radio, the channels and the current position
class LivePage : public PageBase
{
    Q_OBJECT
public:
    explicit LivePage(QWidget* parent = nullptr);
    ~LivePage() override;
    void refresh() override;
    int stage() const override { return 0; }

    void clearContents() override;
    void setRunning(bool running);
    void setMessage(const QString& text);
    void setStatus(const gnss::LiveStatus& status, const std::vector<gnss::LiveChannelStatus>& channels);
    bool frozen() const;

signals:
    void startRequested();
    void stopRequested();
    void settingsRequested();
    void freezeToggled(bool on);
    void prnSelected(int prn);

private:
    void updatePosition();

    Ui::LivePage* ui;
    gnss::LiveStatus status_;
    std::vector<gnss::LiveChannelStatus> channels_;
    QMap<int, QVector<QPointF>> history_;      // C/N0 per satellite
};

} // namespace gui
