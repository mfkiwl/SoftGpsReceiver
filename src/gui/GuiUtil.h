#pragma once

#include "core/Receiver.h"

#include <QColor>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QVector>
#include <QWidget>
#include <vector>

namespace gui {

class DataTable;
class PlotWidget;

// Base class of every inspection page. Pages are laid out in Qt Designer
// (.ui files) and promoted into MainWindow.ui, so they only take a parent;
// the receiver is attached afterwards with setReceiver(). 'refresh' is called
// (in the GUI thread) whenever the results it depends on have changed.
class PageBase : public QWidget
{
    Q_OBJECT
public:
    explicit PageBase(QWidget* parent = nullptr) : QWidget(parent) {}
    // Call once the form is built: clearContents() returns the text on the page to what it is now
    void setReceiver(gnss::Receiver* rx);
    virtual void refresh() = 0;
    // Stage whose results this page shows (refreshed when that stage completes).
    virtual int stage() const = 0;
    // Forgets everything shown from a recording: plots, tables and epoch selectors, and the labels, group titles, plot
    // titles and table columns that pages rewrite to describe their data. Follow it with refresh().
    virtual void clearContents();

protected:
    bool readable(int stage) const;
    gnss::Receiver* rx_ = nullptr;

private:
    struct PlotLabels { PlotWidget* plot; QString title, x, y; };
    struct TableColumns { DataTable* table; QStringList headers; };
    struct LabelText { QLabel* label; QString text; };
    struct GroupTitle { QGroupBox* group; QString title; };
    QVector<PlotLabels> plotLabels_;
    QVector<TableColumns> tableColumns_;
    QVector<LabelText> labelTexts_;
    QVector<GroupTitle> groupTitles_;
};

// Highest pipeline stage whose results may be read from the GUI thread.
// Only ever touched from the GUI thread (see MainWindow).
int readableStage();
void setReadableStage(int s);
// True while a processing stage runs on the worker thread.
bool workerRunning();
void setWorkerRunning(bool on);
// True while the receiver runs on a live stream and the pages show its latest snapshot.
bool liveMode();
void setLiveMode(bool on);

QColor seriesColor(int i);
QColor prnColor(int prn);
// Satellite label for tables and plots: the number for a GPS PRN, "E30" for Galileo
QString satLabel(int id);
// "GPS" or "Galileo"
QString satSystem(int id);
// Name that says which system it is: "GPS PRN 3", "Galileo E30"
QString satFullName(int id);
// Position on a per-satellite bar chart: GPS at its PRN, Galileo after a gap (E01 at 36)
double satAxis(int id);
// Tick text of such a chart: the PRN for GPS, "E05" for Galileo, nothing in the gap
QString satAxisText(double x);
QColor goodColor();
QColor badColor();
QColor warnColor();
QColor neutralColor();

template <typename T>
QVector<double> toQ(const std::vector<T>& v, double scale = 1.0, double offset = 0.0)
{
    QVector<double> r(int(v.size()));
    for (int i = 0; i < int(v.size()); ++i) r[i] = double(v[size_t(i)]) * scale + offset;
    return r;
}

QVector<double> range(int n, double scale = 1.0, double offset = 0.0);
QString fmt(double v, int prec);
QString fmtSci(double v, int prec = 6);
QString deg(double rad, int prec = 6);

QGroupBox* group(const QString& title, QWidget* content);
QLabel* valueLabel();
QString channelLabel(const gnss::TrackingChannel& ch);
// Sets the range of an epoch selector. In live mode a selector left at the newest epoch stays there.
void setEpochRange(QSpinBox* spin, QSlider* slider, int count);

} // namespace gui
