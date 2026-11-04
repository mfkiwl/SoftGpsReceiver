#include "SettingsDialog.h"
#include "ui_SettingsDialog.h"

#include <QCoreApplication>
#include <QGroupBox>
#include <QPushButton>
#include <QTimer>
#include <algorithm>
#include <utility>

using namespace gnss;

namespace gui {

// The combo boxes in SettingsDialog.ui list their items in enum order, so the
// current index is the enum value.

SettingsDialog::SettingsDialog(const ReceiverSettings& s, QWidget* parent)
    : QDialog(parent), ui(new Ui::SettingsDialog), base_(s)
{
    ui->setupUi(this);
    connect(ui->parseButton, &QPushButton::clicked, this, [this] {
        double fs = ui->fsSpin->value() * 1e6, fif = ui->ifSpin->value() * 1e6;
        SampleFormat fmt = SampleFormat(ui->formatCombo->currentIndex());
        ReceiverSettings::parseFileName(base_.fileName, &fs, &fif, &fmt);
        ui->fsSpin->setValue(fs / 1e6);
        ui->ifSpin->setValue(fif / 1e6);
        ui->formatCombo->setCurrentIndex(int(fmt));
    });
    connect(ui->fsSpin, &QDoubleSpinBox::valueChanged, this, &SettingsDialog::updateDerived);
    connect(ui->ifSpin, &QDoubleSpinBox::valueChanged, this, &SettingsDialog::updateDerived);
    connect(ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), &QPushButton::clicked, this, [this] {
        ReceiverSettings d;
        d.fileName = base_.fileName;
        d.samplingFreq = ui->fsSpin->value() * 1e6;
        d.ifFreq = ui->ifSpin->value() * 1e6;
        d.format = SampleFormat(ui->formatCombo->currentIndex());
        load(d);
    });

    // Each piece of equipment's settings sit in a group whose title box shows or hides them, so that the Live tab
    // fits on a screen. The group of the chosen live source opens by itself.
    const std::pair<QGroupBox*, QWidget*> groups[] = {{ui->usrpGroup, ui->usrpBox}, {ui->hackrfGroup, ui->hackrfBox},
                                                      {ui->smGroup, ui->smBox}, {ui->tcpGroup, ui->tcpBox},
                                                      {ui->processingGroup, ui->processingBox}};
    for (const auto& [group, box] : groups) {
        const QMargins margins = group->layout()->contentsMargins();
        // Closed, a group is only its title on a line
        const auto show = [group = group, box = box, margins](bool on) {
            box->setVisible(on);
            group->setFlat(!on);
            group->layout()->setContentsMargins(on ? margins : QMargins());
        };
        show(group->isChecked());
        connect(group, &QGroupBox::toggled, this, [this, show](bool on) {
            show(on);
            // A closed group gives its height back. The layouts have to take the change first, or the size hint is
            // still the old one.
            QTimer::singleShot(0, this, [this] {
                QCoreApplication::sendPostedEvents(nullptr, QEvent::LayoutRequest);
                resize(width(), sizeHint().height());
            });
        });
    }
    connect(ui->liveSourceCombo, &QComboBox::currentIndexChanged, this, &SettingsDialog::openSourceGroup);
    load(s);
}

void SettingsDialog::openSourceGroup()
{
    switch (LiveSourceKind(ui->liveSourceCombo->currentIndex())) {
    case LiveSourceKind::Usrp: ui->usrpGroup->setChecked(true); break;
    case LiveSourceKind::HackRf: ui->hackrfGroup->setChecked(true); break;
    case LiveSourceKind::SignalHound: ui->smGroup->setChecked(true); break;
    case LiveSourceKind::GpsSim: ui->tcpGroup->setChecked(true); break;
    case LiveSourceKind::Replay: ui->processingGroup->setChecked(true); break;      // for the replay speed
    }
}

SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::updateDerived()
{
    const double fs = ui->fsSpin->value() * 1e6, fif = ui->ifSpin->value() * 1e6;
    ui->derivedLabel->setText(tr("%1 samples per ms, %2 samples per chip, IF/fs = %3, fs/4 = %4 MHz")
                                  .arg(fs / 1000.0, 0, 'f', 2).arg(fs / 1.023e6, 0, 'f', 3)
                                  .arg(fif / fs, 0, 'f', 5).arg(fs / 4e6, 0, 'f', 6));
}

void SettingsDialog::load(const ReceiverSettings& s)
{
    ui->formatCombo->setCurrentIndex(int(s.format));
    ui->fsSpin->setValue(s.samplingFreq / 1e6);
    ui->ifSpin->setValue(s.ifFreq / 1e6);
    ui->skipSpin->setValue(int(s.skipBytes));
    ui->msProcessSpin->setValue(s.msToProcess);
    QStringList p;
    for (int v : s.prnList()) p << QString::number(v);
    ui->prnsEdit->setText(p.size() == 32 ? QStringLiteral("1-32") : p.join(','));
    ui->constellationCombo->setCurrentIndex(int(s.constellations));
    ui->galNonCohSpin->setValue(s.galNonCoherent);
    ui->acqStartSpin->setValue(s.acqStartMs);
    ui->acqBandSpin->setValue(s.acqSearchBand);
    ui->acqStepSpin->setValue(s.acqStep);
    ui->acqNonCohSpin->setValue(s.acqNonCoherent);
    ui->acqThrSpin->setValue(s.acqThreshold);
    ui->fineMsSpin->setValue(s.fineFreqMs);
    ui->channelsSpin->setValue(s.numChannels);
    ui->pllBwSpin->setValue(s.pllBandwidth);
    ui->pllZetaSpin->setValue(s.pllDamping);
    ui->fllBwSpin->setValue(s.fllBandwidth);
    ui->fllMsSpin->setValue(s.fllAssistMs);
    ui->dllBwSpin->setValue(s.dllBandwidth);
    ui->dllZetaSpin->setValue(s.dllDamping);
    ui->dllSpacingSpin->setValue(s.dllSpacing);
    ui->galDllSpacingSpin->setValue(s.galDllSpacing);
    ui->aidingCheck->setChecked(s.carrierAiding);
    ui->cn0WinSpin->setValue(s.cn0WindowMs);
    ui->bitStartSpin->setValue(s.bitSyncStartMs);
    ui->navPeriodSpin->setValue(s.navSolPeriodMs);
    ui->elMaskSpin->setValue(s.elevationMask);
    ui->minCn0Spin->setValue(s.minCn0);
    ui->ionoCheck->setChecked(s.useIono);
    ui->tropoCheck->setChecked(s.useTropo);
    ui->earthRotCheck->setChecked(s.useEarthRotation);
    ui->elWeightCheck->setChecked(s.elevationWeighting);
    ui->dopSignCombo->setCurrentIndex(int(s.dopplerSign));
    const LiveSettings& l = s.live;
    ui->liveSourceCombo->setCurrentIndex(int(l.source));
    openSourceGroup();      // also when the index did not change
    ui->deviceArgsEdit->setText(l.deviceArgs);
    ui->centerSpin->setValue(l.centerFreq / 1e6);
    ui->rateSpin->setValue(l.sampleRate / 1e6);
    ui->gainSpin->setValue(l.gainDb);
    ui->antennaEdit->setText(l.antenna);
    ui->subdevEdit->setText(l.subdev);
    ui->clockCombo->setCurrentText(l.clockSource);
    ui->bandwidthSpin->setValue(l.bandwidth / 1e6);
    ui->loOffsetSpin->setValue(l.loOffset / 1e6);
    ui->uhdPathEdit->setText(l.uhdLibrary);
    ui->smPathEdit->setText(l.smLibrary);
    ui->smCenterSpin->setValue(l.smCenter / 1e6);
    ui->smRefSpin->setValue(l.smRefLevel);
    ui->hackrfSerialEdit->setText(l.hackrfSerial);
    ui->hackrfLnaSpin->setValue(l.hackrfLnaGain);
    ui->hackrfVgaSpin->setValue(l.hackrfVgaGain);
    ui->hackrfAmpCheck->setChecked(l.hackrfAmp);
    ui->hackrfPowerCheck->setChecked(l.hackrfAntennaPower);
    ui->hackrfPpmSpin->setValue(l.hackrfPpm);
    ui->hackrfPathEdit->setText(l.hackrfLibrary);
    ui->ringSpin->setValue(l.ringSeconds);
    ui->windowSpin->setValue(l.windowSeconds);
    ui->historySpin->setValue(l.historySeconds);
    ui->updateSpin->setValue(l.updateMs);
    ui->acqIntervalSpin->setValue(l.acqIntervalSec);
    ui->replaySpeedSpin->setValue(l.replaySpeed);
    ui->tcpHostEdit->setText(l.tcpHost);
    ui->tcpPortSpin->setValue(l.tcpPort);
    ui->recordCheck->setChecked(l.record);
    ui->recordDirEdit->setText(l.recordDir);
    updateDerived();
}

ReceiverSettings SettingsDialog::settings() const
{
    ReceiverSettings s = base_;
    s.format = SampleFormat(ui->formatCombo->currentIndex());
    s.samplingFreq = ui->fsSpin->value() * 1e6;
    s.ifFreq = ui->ifSpin->value() * 1e6;
    s.skipBytes = ui->skipSpin->value();
    s.msToProcess = ui->msProcessSpin->value();
    s.acqPrns.clear();
    for (const QString& part : ui->prnsEdit->text().split(',', Qt::SkipEmptyParts)) {
        const QStringList ab = part.trimmed().split('-');
        const int a = ab.value(0).toInt();
        const int b = ab.size() > 1 ? ab.value(1).toInt() : a;
        for (int p = std::max(1, a); p <= std::min(32, b); ++p)
            if (!s.acqPrns.contains(p)) s.acqPrns.append(p);
    }
    if (s.acqPrns.isEmpty())
        for (int p = 1; p <= 32; ++p) s.acqPrns.append(p);
    s.constellations = Constellations(ui->constellationCombo->currentIndex());
    s.galNonCoherent = ui->galNonCohSpin->value();
    s.acqStartMs = ui->acqStartSpin->value();
    s.acqSearchBand = ui->acqBandSpin->value();
    s.acqStep = ui->acqStepSpin->value();
    s.acqNonCoherent = ui->acqNonCohSpin->value();
    s.acqThreshold = ui->acqThrSpin->value();
    s.fineFreqMs = ui->fineMsSpin->value();
    s.numChannels = ui->channelsSpin->value();
    s.pllBandwidth = ui->pllBwSpin->value();
    s.pllDamping = ui->pllZetaSpin->value();
    s.fllBandwidth = ui->fllBwSpin->value();
    s.fllAssistMs = ui->fllMsSpin->value();
    s.dllBandwidth = ui->dllBwSpin->value();
    s.dllDamping = ui->dllZetaSpin->value();
    s.dllSpacing = ui->dllSpacingSpin->value();
    s.galDllSpacing = ui->galDllSpacingSpin->value();
    s.carrierAiding = ui->aidingCheck->isChecked();
    s.cn0WindowMs = ui->cn0WinSpin->value();
    s.bitSyncStartMs = ui->bitStartSpin->value();
    s.navSolPeriodMs = ui->navPeriodSpin->value();
    s.elevationMask = ui->elMaskSpin->value();
    s.minCn0 = ui->minCn0Spin->value();
    s.useIono = ui->ionoCheck->isChecked();
    s.useTropo = ui->tropoCheck->isChecked();
    s.useEarthRotation = ui->earthRotCheck->isChecked();
    s.elevationWeighting = ui->elWeightCheck->isChecked();
    s.dopplerSign = DopplerSign(ui->dopSignCombo->currentIndex());
    LiveSettings& l = s.live;
    l.source = LiveSourceKind(ui->liveSourceCombo->currentIndex());
    l.deviceArgs = ui->deviceArgsEdit->text().trimmed();
    l.centerFreq = ui->centerSpin->value() * 1e6;
    l.sampleRate = ui->rateSpin->value() * 1e6;
    l.gainDb = ui->gainSpin->value();
    l.antenna = ui->antennaEdit->text().trimmed();
    l.subdev = ui->subdevEdit->text().trimmed();
    l.clockSource = ui->clockCombo->currentText().trimmed();
    l.bandwidth = ui->bandwidthSpin->value() * 1e6;
    l.loOffset = ui->loOffsetSpin->value() * 1e6;
    l.uhdLibrary = ui->uhdPathEdit->text().trimmed();
    l.smLibrary = ui->smPathEdit->text().trimmed();
    l.smCenter = ui->smCenterSpin->value() * 1e6;
    l.smRefLevel = ui->smRefSpin->value();
    l.hackrfSerial = ui->hackrfSerialEdit->text().trimmed();
    l.hackrfLnaGain = ui->hackrfLnaSpin->value();
    l.hackrfVgaGain = ui->hackrfVgaSpin->value();
    l.hackrfAmp = ui->hackrfAmpCheck->isChecked();
    l.hackrfAntennaPower = ui->hackrfPowerCheck->isChecked();
    l.hackrfPpm = ui->hackrfPpmSpin->value();
    l.hackrfLibrary = ui->hackrfPathEdit->text().trimmed();
    l.ringSeconds = ui->ringSpin->value();
    l.windowSeconds = ui->windowSpin->value();
    l.historySeconds = ui->historySpin->value();
    l.updateMs = ui->updateSpin->value();
    l.acqIntervalSec = ui->acqIntervalSpin->value();
    l.replaySpeed = ui->replaySpeedSpin->value();
    l.tcpHost = ui->tcpHostEdit->text();
    l.tcpPort = ui->tcpPortSpin->value();
    l.record = ui->recordCheck->isChecked();
    l.recordDir = ui->recordDirEdit->text().trimmed();
    return s;
}

} // namespace gui
