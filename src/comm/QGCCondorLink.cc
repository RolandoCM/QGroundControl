/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


/**
 * @file QGCXCondorLink.cc
 *   Implementation of condor interface based in xplane
 *   @author Rolando Castillo <castillor493@gmail.com>
 *
 */

#include <QTimer>
#include <QList>
#include <QDebug>
#include <QString>
#include <QMutexLocker>
#include <QNetworkInterface>
#include <QHostInfo>
#include <iostream>
#include <Eigen/Eigen>

#include "QGCCondorLink.h"
#include "QGC.h"
#include "UAS.h"
#include "UASInterface.h"
#include "QGCMessageBox.h"

QGCCondorLink::QGCCondorLink(Vehicle* vehicle, QString remoteHost, QHostAddress localHost, quint16 localPort) :
    _vehicle(vehicle),
    remoteHost(QHostAddress("127.0.0.1")),
    remotePort(55280),
    socket(NULL),
    process(NULL),
    terraSync(NULL),
    barometerOffsetkPa(-8.0f),
    airframeID(QGCCondorLink::AIRFRAME_UNKNOWN),
    condorConnected(false),
    simUpdateLast(QGC::groundTimeMilliseconds()),
    simUpdateFirst(0),
    simUpdateLastText(QGC::groundTimeMilliseconds()),
    simUpdateLastGroundTruth(QGC::groundTimeMilliseconds()),
    simUpdateHz(0),
    _sensorHilEnabled(true),
    _useHilActuatorControls(true),
    _should_exit(false)
{
    // We're doing it wrong - because the Qt folks got the API wrong:
    // http://blog.qt.digia.com/blog/2010/06/17/youre-doing-it-wrong/
    moveToThread(this);

    setTerminationEnabled(false);

    this->localHost = localHost;
    this->localPort = localPort/*+mav->getUASID()*/;
    connectState = false;

    this->name = tr("Condor Link (localPort:%1)").arg(localPort);
    setRemoteHost(remoteHost);
    loadSettings();
}

QGCCondorLink::~QGCCondorLink()
{
    storeSettings();
    // Tell the thread to exit
    _should_exit = true;

    if (socket) {
        socket->close();
        socket->deleteLater();
        socket = NULL;
    }
}

void QGCCondorLink::loadSettings()
{
    // Load defaults from settings
    QSettings settings;
    settings.beginGroup("QGC_CONDOR_LINK");
    setRemoteHost(settings.value("REMOTE_HOST", QString("%1:%2").arg(remoteHost.toString()).arg(remotePort)).toString());
   // setVersion(settings.value("CONDOR_VERSION", 10).toInt());
    selectAirframe(settings.value("AIRFRAME", "default").toString());
    _sensorHilEnabled = settings.value("SENSOR_HIL", _sensorHilEnabled).toBool();
    _useHilActuatorControls = settings.value("ACTUATOR_HIL", _useHilActuatorControls).toBool();
    settings.endGroup();
}

void QGCCondorLink::storeSettings()
{
    // Store settings
    QSettings settings;
    settings.beginGroup("QGC_CONDOR_LINK");
    settings.setValue("REMOTE_HOST", QString("%1:%2").arg(remoteHost.toString()).arg(remotePort));
    settings.setValue("AIRFRAME", airframeName);
    settings.setValue("SENSOR_HIL", _sensorHilEnabled);
    settings.setValue("ACTUATOR_HIL", _useHilActuatorControls);
    settings.endGroup();
}



void QGCCondorLink::enableHilActuatorControls(bool enable)
{
    if (enable != _useHilActuatorControls) {
        _useHilActuatorControls = enable;
    }

    /* Only use override for new message and specific airframes */
    MAV_TYPE type = _vehicle->vehicleType();
    float value = 0.0f;
    if (type == MAV_TYPE_VTOL_RESERVED2) {
        value = (enable ? 1.0f : 0.0f);
    }

    sendDataRef("sim/operation/override/override_control_surfaces", value);
    emit useHilActuatorControlsChanged(enable);
}


/**
 * @brief Runs the thread
 *
 **/
void QGCCondorLink::run()
{
    if (!_vehicle) {
        emit statusMessage("No MAV present");
        return;
    }

    if (connectState) {
        emit statusMessage("Already connected");
        return;
    }

    socket = new QUdpSocket(this);
    socket->moveToThread(this);
    connectState = socket->bind(localHost, localPort, QAbstractSocket::ReuseAddressHint);
    if (!connectState) {

        emit statusMessage("Binding socket failed!");

        socket->deleteLater();
        socket = NULL;
        return;
    }

    emit statusMessage(tr("Waiting for Condor.."));
    QObject::connect(socket, &QUdpSocket::readyRead, this, &QGCCondorLink::readBytes);

    connect(_vehicle->uas(), &UAS::hilControlsChanged, this, &QGCCondorLink::updateControls, Qt::QueuedConnection);
    connect(_vehicle, &Vehicle::hilActuatorControlsChanged, this, &QGCCondorLink::updateActuatorControls, Qt::QueuedConnection);

    connect(this, &QGCCondorLink::hilGroundTruthChanged, _vehicle->uas(), &UAS::sendHilGroundTruth, Qt::QueuedConnection);
    connect(this, &QGCCondorLink::hilStateChanged, _vehicle->uas(), &UAS::sendHilState, Qt::QueuedConnection);
    connect(this, &QGCCondorLink::sensorHilGpsChanged, _vehicle->uas(), &UAS::sendHilGps, Qt::QueuedConnection);
    connect(this, &QGCCondorLink::sensorHilRawImuChanged, _vehicle->uas(), &UAS::sendHilSensors, Qt::QueuedConnection);

    _vehicle->uas()->startHil();
#pragma pack(push, 1)
    struct iset_struct
    {
        char b[5];
        int index; // (0->20 in the lsit below)
        char str_ipad_them[16];
        char str_port_them[6];
        char padding[2];
        int use_ip;
    } ip; // to use this option, 0 not to.
#pragma pack(pop)

    ip.b[0] = 'I';
    ip.b[1] = 'S';
    ip.b[2] = 'E';
    ip.b[3] = 'T';
    ip.b[4] = '0';

    QList<QHostAddress> hostAddresses = QNetworkInterface::allAddresses();

    QString localAddrStr;
    QString localPortStr = QString("%1").arg(localPort);

    for (int i = 0; i < hostAddresses.size(); i++)
    {
        // Exclude loopback IPv4 and all IPv6 addresses
        if (hostAddresses.at(i) != QHostAddress("127.0.0.1") && !hostAddresses.at(i).toString().contains(":"))
        {
            localAddrStr = hostAddresses.at(i).toString();
            break;
        }
    }

    ip.index = 0;
    strncpy(ip.str_ipad_them, localAddrStr.toLatin1(), qMin((int)sizeof(ip.str_ipad_them), 16));
    strncpy(ip.str_port_them, localPortStr.toLatin1(), qMin((int)sizeof(ip.str_port_them), 6));
    ip.use_ip = 1;

    qDebug()<<"sois run";
    writeBytesSafe((const char*)&ip, sizeof(ip));

    /* Call function which makes sure individual control override is enabled/disabled */
    enableHilActuatorControls(_useHilActuatorControls);

    _should_exit = false;
    while(!_should_exit) {
        QCoreApplication::processEvents();
        QGC::SLEEP::msleep(5);
           // qDebug()<<"funciona 222";
    }

    disconnect(_vehicle->uas(), &UAS::hilControlsChanged, this, &QGCCondorLink::updateControls);

    disconnect(this, &QGCCondorLink::hilGroundTruthChanged, _vehicle->uas(), &UAS::sendHilGroundTruth);
    disconnect(this, &QGCCondorLink::hilStateChanged, _vehicle->uas(), &UAS::sendHilState);
    disconnect(this, &QGCCondorLink::sensorHilGpsChanged, _vehicle->uas(), &UAS::sendHilGps);
    disconnect(this, &QGCCondorLink::sensorHilRawImuChanged, _vehicle->uas(), &UAS::sendHilSensors);
    connectState = false;

    disconnect(socket, &QUdpSocket::readyRead, this, &QGCCondorLink::readBytes);

    socket->close();
    socket->deleteLater();
    socket = NULL;

    emit simulationDisconnected();
    emit simulationConnected(false);
}

void QGCCondorLink::setPort(int localPort)
{
    this->localPort = localPort;
    disconnectSimulation();
    connectSimulation();
}

void QGCCondorLink::processError(QProcess::ProcessError err)
{
    QString msg;
    
    switch(err) {
        case QProcess::FailedToStart:
            msg = tr("Condor Failed to start. Please check if the path and command is correct");
            break;
            
        case QProcess::Crashed:
            msg = tr("Condor crashed. This is an Condor-related problem, check for Condor upgrade.");
            break;
            
        case QProcess::Timedout:
            msg = tr("Condor start timed out. Please check if the path and command is correct");
            break;
            
        case QProcess::ReadError:
        case QProcess::WriteError:
            msg = tr("Could not communicate with Condor. Please check if the path and command are correct");
            break;
            
        case QProcess::UnknownError:
        default:
            msg = tr("Condor error occurred. Please check if the path and command is correct.");
            break;
    }
    
    
    QGCMessageBox::critical(tr("Condor HIL"), msg);
}

QString QGCCondorLink::getRemoteHost()
{
    return QString("%1:%2").arg(remoteHost.toString()).arg(remotePort);
}

/**
 * @param newHost Hostname in standard formatting, e.g. localhost:14551 or 192.168.1.1:14551
 */
void QGCCondorLink::setRemoteHost(const QString& newHost)
{
    if (newHost.length() == 0)
        return;

    if (newHost.contains(":"))
    {
        QHostInfo info = QHostInfo::fromName(newHost.split(":").first());
        if (info.error() == QHostInfo::NoError)
        {
            // Add newHost
            QList<QHostAddress> newHostAddresses = info.addresses();
            QHostAddress address;
            for (int i = 0; i < newHostAddresses.size(); i++)
            {
                // Exclude loopback IPv4 and all IPv6 addresses
                if (!newHostAddresses.at(i).toString().contains(":"))
                {
                    address = newHostAddresses.at(i);
                }
            }
            remoteHost = address;
            // Set localPort according to user input
            remotePort = newHost.split(":").last().toInt();
        }
    }
    else
    {
        QHostInfo info = QHostInfo::fromName(newHost);
        if (info.error() == QHostInfo::NoError)
        {
            // Add newHost
            remoteHost = info.addresses().first();
            if (remotePort == 0) remotePort = 55286;
        }
    }

    if (isConnected())
    {
        disconnectSimulation();
        connectSimulation();
    }

    emit remoteChanged(QString("%1:%2").arg(remoteHost.toString()).arg(remotePort));
}

void QGCCondorLink::updateControls(quint64 time, float rollAilerons, float pitchElevator, float yawRudder, float throttle, quint8 systemMode, quint8 navMode)
{
    //qDebug()<<"Aile: "<<rollAilerons<< "piEle: "<<pitchElevator<<"yawR: "<< yawRudder<< "thro: " << throttle;
    /* Only use HIL_CONTROL when the checkbox is unchecked */
    if (_useHilActuatorControls) {
        return;
    }
    #pragma pack(push, 1)
    struct payload {
        char b[5];
        int index;
        float f[8];
    } p;
    #pragma pack(pop)

    p.b[0] = 'D';
    p.b[1] = 'A';
    p.b[2] = 'T';
    p.b[3] = 'A';
    p.b[4] = '\0';

    Q_UNUSED(time);
    Q_UNUSED(systemMode);
    Q_UNUSED(navMode);

    if (_vehicle->vehicleType() == MAV_TYPE_QUADROTOR
        || _vehicle->vehicleType() == MAV_TYPE_HEXAROTOR
        || _vehicle->vehicleType() == MAV_TYPE_OCTOROTOR)
    {
        qDebug() << "MAV_TYPE_QUADROTOR";

        // Individual effort will be provided directly to the actuators on condor quadrotor.
        p.f[0] = yawRudder;
        p.f[1] = rollAilerons;
        p.f[2] = throttle;
        p.f[3] = pitchElevator;

        // Direct throttle control
        p.index = 25;
       // writeBytesSafe((const char*)&dataCondor, dataCondor);
    }
    else
    {
        // direct pass-through, normal fixed-wing.
        p.f[0] = -pitchElevator;
        p.f[1] = rollAilerons;
        p.f[2] = yawRudder;

        // Ail / Elevon / Rudder

        // Send to group 12
        p.index = 12;
        writeBytesSafe((const char*)&p, sizeof(p));

        // Send to group 8, which equals manual controls
        p.index = 8;
        writeBytesSafe((const char*)&p, sizeof(p));

        // Send throttle to all four motors
        p.index = 25;
        memset(p.f, 0, sizeof(p.f));
        p.f[0] = throttle;
        p.f[1] = throttle;
        p.f[2] = throttle;
        p.f[3] = throttle;
        writeBytesSafe((const char*)&p, sizeof(p));
    }
}
/** @todo nairbreak no tiene asignacion de variable por el momento*/
void QGCCondorLink::updateActuatorControls(float ctl_0, float ctl_1, float ctl_2)
{
    QByteArray actuatorControls;
    if (!_useHilActuatorControls) {
        qDebug() << "received HIL_ACTUATOR_CONTROLS but not using it";
        return;
    }
    actuatorControls.append("ailerons="+ QString::number(ctl_0)+"\r\nelevator="+QString::number(-ctl_1)+"\r\nrudder="+QString::number(ctl_2)+"\r\nnairbreak=0.0\r\n");
    emit _invokeWriteBytes(actuatorControls);
    //qDebug()<<"Actuator controls: "<< actuatorControls;

}

Eigen::Matrix3f euler_to_wRoC(double yaw, double pitch, double roll) {
  double c__ = cos(yaw);
  double _c_ = cos(pitch);
  double __c = cos(roll);
  double s__ = sin(yaw);
  double _s_ = sin(pitch);
  double __s = sin(roll);
  double cc_ = c__ * _c_;
  double cs_ = c__ * _s_;
  double sc_ = s__ * _c_;
  double ss_ = s__ * _s_;
  double c_c = c__ * __c;
  double c_s = c__ * __s;
  double s_c = s__ * __c;
  double s_s = s__ * __s;
  double _cc = _c_ * __c;
  double _cs = _c_ * __s;
  double csc = cs_ * __c;
  double css = cs_ * __s;
  double ssc = ss_ * __c;
  double sss = ss_ * __s;
  Eigen::Matrix3f wRo;
  wRo <<
    cc_  , css-s_c,  csc+s_s,
    sc_  , sss+c_c,  ssc-c_s,
    -_s_  ,     _cs,      _cc;
  return wRo;
}

void QGCCondorLink::_writeBytes(const QByteArray data)
{
    if (data.isEmpty()) return;

    // If socket exists and is connected, transmit the data
    if (socket && connectState)
    {
        socket->writeDatagram(data, remoteHost, remotePort);
    }
}

/**
 * @brief Read all pending packets from the interface.
 **/
void QGCCondorLink::readBytes()
{
    // Only emit updates on attitude message
    bool emitUpdate = false;
    quint16 fields_changed = 0;

    const qint64 maxLength = 65536;
    char data[maxLength];
    QHostAddress sender;
    quint16 senderPort;
    unsigned int s = socket->pendingDatagramSize();
    if (s > maxLength) std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size: " << s << std::endl;
    socket->readDatagram(data, maxLength, &sender, &senderPort);
    if (s > maxLength) {
    	std::string headStr = std::string(data, data+5);
    	std::cerr << __FILE__ << __LINE__ << " UDP datagram header: " << headStr << std::endl;
    }
    // Calculate the number of data segments
    bool oldConnectionState = condorConnected;
    char *variable;

    //qDebug()<<data;
    if(data[0]=='t')
    {

        //bloque en obserbación
        condorConnected = true;
        if (oldConnectionState != condorConnected) {
            simUpdateFirst = QGC::groundTimeMilliseconds();
        }
            //1 para ind_airspeed, true_airspeed, groundspeed

            /*
             * ind_airspeed == airspeed
             * true_airspeed == trueairspeed
             * groundspeed ==   ##### not asigned
             * */

               //  ind_airspeed=data.IndexOf("airspeed");
                 variable = strstr(data,"airspeed");
                   ind_airspeed = Convert(variable+8, 10);
                variable = strstr(data,"trueairspeed");
                    true_airspeed = Convert(variable+12, 10);  //verificar unidades C-Q ###########################

                //not asigned or calculs
                groundspeed=sqrt(vx*vx+vy*vy);


                 /*
                  * axC=ay, ayC=-ax azC=-az son variables de condor
                  *    axf = ay;
                       ayf = -ax;
                       azf = -az;
                        Eigen::Vector3f acc(axf, ayf, azf-9.80665f);
                  * */
                variable = strstr(data,"ax");
                ayC=-Convert(variable+2, 10);

                variable = strstr(data,"ay");
                axC = Convert(variable+2, 10);

                variable = strstr(data,"az");
                azC=-Convert(variable+2, 10);
                //qDebug()<<"ax: "<<ayC<<"ay: "<<axC<<"az: "<<azC;

                Eigen::Vector3f acc(axC, ayC, azC-9.80665f);
                Eigen::Matrix3f R = euler_to_wRoC(yaw, pitch, roll);
                Eigen::Vector3f accb = R.transpose().eval() * acc;

                xacc = accb[0];
                yacc = accb[1];
                zacc = accb[2];

                fields_changed |= (1 << 0) | (1 << 1) | (1 << 2);
                emitUpdate=true;


            // index 6 abs_pressure, temperature
                // variable = strstr(data,"abs_presure");
                abs_pressure = 240.01f;// Convert(variable+8, 8);
                 temperature=36;
                 fields_changed |= (1 << 9) | (1 << 12);


            // index 16  pitchspeed, rollspeed, yawspeed
                 variable = strstr(data,"pitchrate");
                 pitchspeed = Convert(variable+9,10);

                 variable = strstr(data,"rollrate");
                 rollspeed = -Convert(variable+8,10);


                 variable = strstr(data,"yawrate");
                 yawspeed = Convert(variable+7,10);

                 variable = strstr(data, "vario");
                 vario = Convert(variable+5,10);
                 //qDebug()<<"QGC Condor variometer: "<< vario;

                 fields_changed |= (1 << 3) | (1 << 4) | (1 << 5);
                 emitUpdate = true;



            // indext 17    pitch, roll, yaw  yawmag xmag, ymag, zmag
                 variable = strstr(data,"pitch=");
                 pitch = Convert(variable+5,10);

                 variable = strstr(data,"bank");
                 roll = -Convert(variable+4,10);

                 variable = strstr(data,"yaw=");
                 yaw = Convert(variable+3,10);

                 variable = strstr(data,"compass");
                 yawmag = Convert(variable+7,10) / 180.0f * M_PI;

                 //--------------------------------------------------------------
                 //qDebug()<<"pitch: "<<pitch<<"bank: "<<roll<<"yaw: "<<yaw;

                 if (yaw > M_PI) {
                     yaw -= 2.0f * static_cast<float>(M_PI);
                 }
                 if (yaw < -M_PI) {
                     yaw += 2.0f * static_cast<float>(M_PI);
                 }

                 if (yawmag > M_PI) {
                     yawmag -= 2.0f * static_cast<float>(M_PI);
                 }
                 if (yawmag < -M_PI) {
                     yawmag += 2.0f * static_cast<float>(M_PI);
                 }
                 xmag = cos(-yawmag) * 0.25f;
                 ymag = sin(-yawmag) * 0.25f;
                 zmag = 0.45f;

                 fields_changed |= (1 << 6) | (1 << 7) | (1 << 8);


                 double cosPhi = cos(roll);
                 double sinPhi = sin(roll);
                 double cosThe = cos(pitch);
                 double sinThe = sin(pitch);
                 double cosPsi = cos(0.0);
                 double sinPsi = sin(0.0);

                 float dcm[3][3];

                 dcm[0][0] = cosThe * cosPsi;
                 dcm[0][1] = -cosPhi * sinPsi + sinPhi * sinThe * cosPsi;
                 dcm[0][2] = sinPhi * sinPsi + cosPhi * sinThe * cosPsi;

                 dcm[1][0] = cosThe * sinPsi;
                 dcm[1][1] = cosPhi * cosPsi + sinPhi * sinThe * sinPsi;
                 dcm[1][2] = -sinPhi * cosPsi + cosPhi * sinThe * sinPsi;

                 dcm[2][0] = -sinThe;
                 dcm[2][1] = sinPhi * cosThe;
                 dcm[2][2] = cosPhi * cosThe;

                 Eigen::Matrix3f m = Eigen::Map<Eigen::Matrix3f>((float*)dcm).eval();

                 Eigen::Vector3f mag(xmag, ymag, zmag);

                 Eigen::Vector3f magbody = m * mag;

                 xmag = magbody(0);
                 ymag = magbody(1);
                 zmag = magbody(2);


                 emitUpdate = true;


            //index 20    lat, lon, alt, alt_agl
                variable = strstr(data, "lat");
                    lat=Convert(variable+3, 10);
                variable = strstr(data, "lon");
                    lon = Convert(variable+3,10);

                variable = strstr(data,"altitude");
                    pressure_alt=Convert(variable+8,10);
                variable = strstr(data,"height");
                    alt_agl=Convert(variable+6,10);
                variable = strstr(data,"altitude"); //altitude is equals to z
                    alt=Convert(variable+8,10);

               // qDebug()<<"lat: "<<lat<<" lon: "<<lon<<" alt "<<alt;


                 variable = strstr(data,"vx");
                     vy = -Convert(variable+2,10); // vx de condor es asignado a vy, (vy=-vxCondor)
                 variable = strstr(data,"vy");
                     vx = Convert(variable+2,10);  //vy de condor es asignado a vx,  (vx = vyCondor)
                 variable = strstr(data,"vz");
                     vz = -Convert(variable+2,10);  //vz de condor es asignado a -vz,  (vz = -vzCondor)

            emitUpdate=true;


    }
    else
    {
        qDebug() << "UNKNOWN PACKET:" << data;
    }

    // Wait for 0.5s before actually using the data, so that all fields are filled
    if (QGC::groundTimeMilliseconds() - simUpdateFirst < 500) {
        return;
    }

  //(   Send updated state
        if (emitUpdate && (QGC::groundTimeMilliseconds() - simUpdateLast) > 2)
        {
            simUpdateHz = simUpdateHz * 0.9f + 0.1f * (1000.0f / (QGC::groundTimeMilliseconds() - simUpdateLast));
            if (QGC::groundTimeMilliseconds() - simUpdateLastText > 2000) {
                emit statusMessage(tr("Receiving from Condor at %1 Hz").arg(static_cast<int>(simUpdateHz)));
                // Reset lowpass with current value
                simUpdateHz = (1000.0f / (QGC::groundTimeMilliseconds() - simUpdateLast));
                // Set state
                simUpdateLastText = QGC::groundTimeMilliseconds();
            }

            simUpdateLast = QGC::groundTimeMilliseconds();

            if (_sensorHilEnabled)
            {
                diff_pressure = (ind_airspeed * ind_airspeed * 1.225f) / 2.0f;



                // set pressure alt to changed
                fields_changed |= (1 << 11);


                emit hilStateChanged(QGC::groundTimeUsecs(), roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, vario, lat, lon, alt,
                                     vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc);

//                emit sensorHilRawImuChanged(QGC::groundTimeUsecs(), xacc, yacc, zacc, rollspeed, pitchspeed, yawspeed,
//                                            xmag, ymag, zmag, abs_pressure, diff_pressure / 100.0, pressure_alt, temperature, fields_changed);

              //  qDebug()<<"sensorHilEnable";

                // XXX make these GUI-configurable and add randomness
                int gps_fix_type = 3;
                float eph = 0.3f;
                float epv = 0.6f;
                float vel = sqrt(vx*vx + vy*vy + vz*vz);
                float cog = atan2(vy, vx);
                int satellites = 8;

                emit sensorHilGpsChanged(QGC::groundTimeUsecs(), lat, lon, alt, gps_fix_type, eph, epv, vel, vx, vy, vz, cog, satellites);

            } else {
                emit hilStateChanged(QGC::groundTimeUsecs(), roll, pitch, yaw, rollspeed,
                                     pitchspeed, yawspeed, vario, lat, lon, alt,
                                     vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc);
            }

            // Limit ground truth to 25 Hz
            if (QGC::groundTimeMilliseconds() - simUpdateLastGroundTruth > 40) {
                emit hilGroundTruthChanged(QGC::groundTimeUsecs(), roll, pitch, yaw, rollspeed,
                                           pitchspeed, yawspeed, lat, lon, alt,
                                           vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc);
                simUpdateLastGroundTruth = QGC::groundTimeMilliseconds();

//                qDebug()<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<< " rollspeed: "<< rollspeed<< " pitchspeed: "<<pitchspeed<< " yawspeed: "<<yawspeed
//                       <<" lat: "<<lat<<" lon: "<<lon<<" alt: "<< alt<< " vx: "<<vx<< " vy: "<<vy<<" vz: "<<vz<<" ind_airspeed: "<< ind_airspeed<< " true_airspeed: "
//                      <<true_airspeed<<" xacc: "<<xacc<<" yacc: "<<yacc<< " zacc: "<<zacc;
            }
        }

        if (!oldConnectionState && condorConnected)
        {
            emit statusMessage(tr("Receiving from Condor."));
        }
}




float QGCCondorLink::Convert(char *i,int maxLenght)
{
    QString modulo="";
    unsigned cont=1;
    float valueFloat=0.0f;
    while(cont!=maxLenght)
    {
            modulo.append(*(i+cont) );
            cont++;
    }
    valueFloat=modulo.toFloat();
    return valueFloat;
}


/**
 * @brief Get the number of bytes to read.
 *
 * @return The number of bytes to read
 **/
qint64 QGCCondorLink::bytesAvailable()
{
    return socket->pendingDatagramSize();
}

/**
 * @brief Disconnect the connection.
 *
 * @return True if connection has been disconnected, false if connection couldn't be disconnected.
 **/
bool QGCCondorLink::disconnectSimulation()
{
    if (connectState)
    {
        _should_exit = true;
    } else {
        emit simulationDisconnected();
        emit simulationConnected(false);
    }

    return !connectState;
}

void QGCCondorLink::selectAirframe(const QString& condor)
{
    airframeName = condor;

    if (condor.contains("QRO"))
    {
        if (condor.contains("MK") && airframeID != AIRFRAME_QUAD_X_MK_10INCH_I2C)
        {
            airframeID = AIRFRAME_QUAD_X_MK_10INCH_I2C;
            emit airframeChanged("QRO_X / MK");
        }
        else if (condor.contains("ARDRONE") && airframeID != AIRFRAME_QUAD_X_ARDRONE)
        {
            airframeID = AIRFRAME_QUAD_X_ARDRONE;
            emit airframeChanged("QRO_X / ARDRONE");
        }
        else
        {
            bool changed = (airframeID != AIRFRAME_QUAD_DJI_F450_PWM);
            airframeID = AIRFRAME_QUAD_DJI_F450_PWM;
            if (changed) emit airframeChanged("QRO_X / DJI-F450 / PWM");
        }
    }
    else
    {
        bool changed = (airframeID != AIRFRAME_UNKNOWN);
        airframeID = AIRFRAME_UNKNOWN;
        if (changed) emit airframeChanged("Condor default");
    }
}

void QGCCondorLink::setPositionAttitude(double lat, double lon, double alt, double roll, double pitch, double yaw)
{
    qDebug()<<"sois setPositionAttitude";
    #pragma pack(push, 1)
    struct VEH1_struct
    {
        char header[5];
        quint32 p;
        double lat_lon_ele[3];
        float psi_the_phi[3];
        float gear_flap_vect[3];
    } pos;
    #pragma pack(pop)

    pos.header[0] = 'V';
    pos.header[1] = 'E';
    pos.header[2] = 'H';
    pos.header[3] = '1';
    pos.header[4] = '0';
    pos.p = 0;
    pos.lat_lon_ele[0] = lat;
    pos.lat_lon_ele[1] = lon;
    pos.lat_lon_ele[2] = alt;
    pos.psi_the_phi[0] = roll;
    pos.psi_the_phi[1] = pitch;
    pos.psi_the_phi[2] = yaw;
    pos.gear_flap_vect[0] = 0.0f;
    pos.gear_flap_vect[1] = 0.0f;
    pos.gear_flap_vect[2] = 0.0f;

    writeBytesSafe((const char*)&pos, sizeof(pos));

}

/**
 * Sets a random position with an offset of max 1/1000 degree
 * and max 100 m altitude
 */
void QGCCondorLink::setRandomPosition()
{
    // Initialize generator
    srand(0);

    double offLat = rand() / static_cast<double>(RAND_MAX) / 500.0 + 1.0/500.0;
    double offLon = rand() / static_cast<double>(RAND_MAX) / 500.0 + 1.0/500.0;
    double offAlt = rand() / static_cast<double>(RAND_MAX) * 200.0 + 100.0;

    if (_vehicle->altitudeAMSL()->rawValue().toDouble() + offAlt < 0)
    {
        offAlt *= -1.0;
    }

    setPositionAttitude(_vehicle->latitude() + offLat,
                        _vehicle->longitude() + offLon,
                        _vehicle->altitudeAMSL()->rawValue().toDouble() + offAlt,
                        _vehicle->roll()->rawValue().toDouble(),
                        _vehicle->pitch()->rawValue().toDouble(),
                        _vehicle->uas()->getYaw());
}

void QGCCondorLink::setRandomAttitude()
{
    // Initialize generator
    srand(0);

    double roll = rand() / static_cast<double>(RAND_MAX) * 2.0 - 1.0;
    double pitch = rand() / static_cast<double>(RAND_MAX) * 2.0 - 1.0;
    double yaw = rand() / static_cast<double>(RAND_MAX) * 2.0 - 1.0;

    setPositionAttitude(_vehicle->latitude(),
                        _vehicle->longitude(),
                        _vehicle->altitudeAMSL()->rawValue().toDouble(),
                        roll,
                        pitch,
                        yaw);
}

/**
 * @brief Connect the connection.
 *
 * @return True if connection has been established, false if connection couldn't be established.
 **/
bool QGCCondorLink::connectSimulation()
{
    if (connectState) {
        qDebug() << "Simulation already active";
    } else {
        qDebug() << "STARTING Condor LINK, CONNECTING TO" << remoteHost << ":" << remotePort;
        // XXX Hack
        storeSettings();

        start(HighPriority);
    }

    return true;
}

/**
 * @brief Check if connection is active.
 *
 * @return True if link is connected, false otherwise.
 **/
bool QGCCondorLink::isConnected()
{
    return connectState;
}

QString QGCCondorLink::getName()
{
    return name;
}

void QGCCondorLink::setName(QString name)
{
    this->name = name;
    //    emit nameChanged(this->name);
}

void QGCCondorLink::sendDataRef(QString ref, float value)
{
    qDebug()<<"sois sendDataRef";
    #pragma pack(push, 1)
    struct payload {
        char b[5];
        float value;
        char name[500];
    } dref;
    #pragma pack(pop)

    dref.b[0] = 'D';
    dref.b[1] = 'R';
    dref.b[2] = 'E';
    dref.b[3] = 'F';
    dref.b[4] = '0';

    /* Set value */
    dref.value = value;

    /* Fill name with zeroes */
    memset(dref.name, 0, sizeof(dref.name));

    /* Set dref name */

    /* Send command */
    QByteArray ba = ref.toUtf8();
    if (ba.length() > 500) {
        return;
    }

    for (int i = 0; i < ba.length(); i++) {
        dref.name[i] = ba.at(i);
    }
    writeBytesSafe((const char*)&dref, sizeof(dref));
}
