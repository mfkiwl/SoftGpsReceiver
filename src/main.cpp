#include "gui/MainWindow.h"

#include <QApplication>
#include <QIcon>
#include <QStyleFactory>

// Usage:
//   SoftGpsReceiver [recording.bin]
//   SoftGpsReceiver recording.bin --screenshots <dir>   (process, save PNGs of every page, exit)
//   SoftGpsReceiver recording.bin --screenshots <dir> --live <seconds>   (the same during a real-time replay)
//   SoftGpsReceiver --screenshots <dir> --live <seconds> --usrp          (the same with the radio)
int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("SoftGpsReceiver"));
    QCoreApplication::setApplicationName(QStringLiteral("SoftGpsReceiver"));
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0"));
    app.setStyle(QStyleFactory::create(QStringLiteral("Fusion")));
    app.setWindowIcon(QIcon(QStringLiteral(":/resources/SoftGpsReceiver.ico")));

    QString file, shots;
    int liveSeconds = 0;
    bool usrp = false;
    const QStringList args = app.arguments();
    for (int i = 1; i < args.size(); ++i) {
        if (args[i] == QLatin1String("--screenshots") && i + 1 < args.size()) shots = args[++i];
        else if (args[i] == QLatin1String("--live") && i + 1 < args.size()) liveSeconds = args[++i].toInt();
        else if (args[i] == QLatin1String("--usrp")) usrp = true;
        else if (!args[i].startsWith(QLatin1String("--"))) file = args[i];
    }

    if (!shots.isEmpty()) app.setFont(QFont(QStringLiteral("Segoe UI"), 9));   // deterministic font for offscreen rendering

    gui::MainWindow w;
    if (!shots.isEmpty()) {
        w.resize(1700, 1050);
        w.show();
        if (!usrp) w.openInitialFile(file);
        w.captureScreenshots(shots, liveSeconds, usrp);
        return 0;
    }
    w.show();
    w.openInitialFile(file);
    return app.exec();
}
