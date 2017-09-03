#include "QGCHilCondorConfiguration.h"
#include "ui_QGCHilCondorConfiguration.h"
#include "QGCCondorLink.h"
#include "QGCHilConfiguration.h"

QGCHilCondorConfiguration::QGCHilCondorConfiguration(QGCHilLink* link, QGCHilConfiguration *parent) :
    QWidget(parent),
    ui(new Ui::QGCHilCondorConfiguration)
{
    ui->setupUi(this);
    this->link = link;

    connect(ui->startButton, &QPushButton::clicked, this, &QGCHilCondorConfiguration::toggleSimulation);

    connect(ui->hostComboBox, static_cast<void (QComboBox::*)(const QString&)>(&QComboBox::activated),
            link, &QGCHilLink::setRemoteHost);

    connect(link, &QGCHilLink::remoteChanged, ui->hostComboBox, &QComboBox::setEditText);
    connect(link, &QGCHilLink::statusMessage, parent, &QGCHilConfiguration::receiveStatusMessage);

//    connect(mav->getHILSimulation(), SIGNAL(statusMessage(QString)), this, SLOT(receiveStatusMessage(QString)));
//    connect(ui->simComboBox, SIGNAL(activated(QString)), mav->getHILSimulation(), SLOT(setVersion(QString)));

    ui->startButton->setText(tr("Connect"));

    QGCCondorLink* condor = dynamic_cast<QGCCondorLink*>(link);

    if (condor)
    {
//        connect(ui->randomAttitudeButton, SIGNAL(clicked()), link, SLOT(setRandomAttitude()));
//        connect(ui->randomPositionButton, SIGNAL(clicked()), link, SLOT(setRandomPosition()));

        //ui->airframeComboBox->setCurrentIndex(link->getAirFrameIndex());
        //connect(ui->airframeComboBox, SIGNAL(activated(QString)), link, SLOT(selectAirframe(QString)));
        // XXX not implemented yet
        //ui->airframeComboBox->hide();
        ui->sensorHilCheckBox->setChecked(condor->sensorHilEnabled());
        ui->useHilActuatorControlsCheckBox->setChecked(true);
        connect(condor, &QGCCondorLink::sensorHilChanged, ui->sensorHilCheckBox, &QCheckBox::setChecked);
        connect(ui->sensorHilCheckBox, &QCheckBox::clicked, condor, &QGCCondorLink::enableSensorHIL);
        connect(condor, &QGCCondorLink::useHilActuatorControlsChanged, ui->useHilActuatorControlsCheckBox, &QCheckBox::setChecked);
        connect(ui->useHilActuatorControlsCheckBox, &QCheckBox::clicked, condor, &QGCCondorLink::enableHilActuatorControls);

        connect(link, static_cast<void (QGCHilLink::*)(int)>(&QGCHilLink::versionChanged),
                this, &QGCHilCondorConfiguration::setVersion);
    }

    ui->hostComboBox->clear();
    ui->hostComboBox->addItem(link->getRemoteHost());


}

void QGCHilCondorConfiguration::setVersion(int version)
{
    Q_UNUSED(version);
}

void QGCHilCondorConfiguration::toggleSimulation(bool connect)
{
    if (!link) {
        return;
    }

    Q_UNUSED(connect);
    if (!link->isConnected())
    {
        link->setRemoteHost(ui->hostComboBox->currentText());
        link->connectSimulation();
        ui->startButton->setText(tr("Disconnect"));
    }
    else
    {
        link->disconnectSimulation();
        ui->startButton->setText(tr("Connect"));
    }
}

QGCHilCondorConfiguration::~QGCHilCondorConfiguration()
{
    delete ui;
}
