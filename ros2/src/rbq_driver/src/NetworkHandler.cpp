#include "NetworkHandler.h"

#include <QCoreApplication>
#include <QDebug>
#include <QTimer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QNetworkDatagram>

using namespace RBQ_SDK;

QHostAddress hostAddress;

QTcpSocket *tcpSocket = nullptr;
QUdpSocket *udpSocket = nullptr;
QTimer *timer = nullptr;
QTimer *autoConnector = nullptr;

NetworkHandler::NetworkHandler(QObject *parent, const std::string &host, const int &_commFrequency)
    : QObject{parent}
    , m_motionFeedbackFrequency{_commFrequency}
{
    hostAddress.setAddress(host.data());
    connect();

    autoConnector = new QTimer(this);
    QObject::connect(autoConnector, &QTimer::timeout, this, &NetworkHandler::autoConnect);
    autoConnector->start(1000/1);

    QObject::connect(this, &NetworkHandler::sendTcpData, this, &NetworkHandler::_sendTcpData);
    QObject::connect(this, &NetworkHandler::sendUdpData, this, &NetworkHandler::_sendUdpData);
}

NetworkHandler::~NetworkHandler()
{
    QObject::disconnect(autoConnector, &QTimer::timeout, this, &NetworkHandler::autoConnect);
    delete autoConnector;
    disconnect();

    if(nullptr != m_robotState) {
        delete m_robotState;
    }
    if(nullptr != m_legStateArray) {
        delete m_legStateArray;
    }

    QObject::disconnect(this, &NetworkHandler::sendTcpData, this, &NetworkHandler::_sendTcpData);
    QObject::disconnect(this, &NetworkHandler::sendUdpData, this, &NetworkHandler::_sendUdpData);
}

void NetworkHandler::connect()
{
    disconnect();
    qDebug() << "Connecting to host: " << hostAddress.toString() << ":" << RBQ_SDK::PORT_ROBOT_MOTION_TCP;
    if(nullptr == tcpSocket) {
        tcpSocket = new QTcpSocket(this);
        QObject::connect(tcpSocket, &QTcpSocket::connected, this, &NetworkHandler::onConnected);
        QObject::connect(tcpSocket, &QTcpSocket::disconnected, this, &NetworkHandler::onDisconnected);
    }
    tcpSocket->connectToHost(hostAddress, RBQ_SDK::PORT_ROBOT_MOTION_TCP, QAbstractSocket::ReadWrite);

    udpSocket = new QUdpSocket(this);
    udpSocket->connectToHost(hostAddress, PORT_ROBOT_MOTION_UDP, QAbstractSocket::ReadWrite);
    QObject::connect(udpSocket, &QUdpSocket::readyRead, this, &NetworkHandler::readSocketMotionDatagram);
    setMotionFeedbackFrequency(m_motionFeedbackFrequency);
}

void NetworkHandler::disconnect()
{
    // setMotionFeedbackFrequency(0);
    onDisconnected();
    if(tcpSocket != nullptr) {
        QObject::disconnect(tcpSocket, &QTcpSocket::connected, this, &NetworkHandler::onConnected);
        QObject::disconnect(tcpSocket, &QTcpSocket::disconnected, this, &NetworkHandler::onDisconnected);
        tcpSocket->disconnect(); tcpSocket->deleteLater(); tcpSocket = nullptr;
    }
    if(nullptr != udpSocket) {
        if(udpSocket->isOpen()) {
            udpSocket->close();
        }
        udpSocket->deleteLater(); udpSocket = nullptr;
    }
}

void NetworkHandler::sendTCP(const QByteArray &msg)
{
    // Q_EMIT sendTcpData(msg);

    _sendTcpData(msg);
}

void NetworkHandler::sendUDP(const QByteArray &msg)
{
    // Q_EMIT sendUdpData(msg);

    _sendUdpData(msg);
}

void NetworkHandler::setMotionFeedbackFrequency(const int &newFrequency)
{
    m_motionFeedbackFrequency = newFrequency;
    if(newFrequency) {
        if(nullptr == timer) {
            timer = new QTimer(this);
            QObject::connect(timer, &QTimer::timeout, this, &NetworkHandler::sendRequestMotionFeedback);
        }
        timer->stop();
        timer->start(1000/m_motionFeedbackFrequency);
    } else {
        if(nullptr != timer) {
            if(timer->isActive()) {
                timer->stop();
            }
            QObject::disconnect(timer, &QTimer::timeout, this, &NetworkHandler::sendRequestMotionFeedback);
            timer->deleteLater(); timer = nullptr;
        }
    }
}

void NetworkHandler::readSocketMotionDatagram()
{
    if(nullptr == udpSocket) {
        return;
    } else if(!udpSocket->hasPendingDatagrams()) {
        return;
    }
    QHostAddress sender_; quint16 senderPort_;
    QByteArray buffer_;
    buffer_.resize(udpSocket->pendingDatagramSize());
    udpSocket->readDatagram(buffer_.data(), buffer_.size(), &sender_, &senderPort_);

    if(buffer_.size() == sizeof(RBQ_SDK::Motion::RobotState_t)) {
        RBQ_SDK::Motion::RobotState_t robotState_;
        memcpy((char *)&robotState_, buffer_.data(), sizeof(RBQ_SDK::Motion::RobotState_t));
        buffer_.remove(0, buffer_.size());
        this->setRobotState(robotState_);
    } else if(buffer_.size() == sizeof(RBQ_SDK::Motion::LegStateArray_t)) {
        RBQ_SDK::Motion::LegStateArray_t legStateArray_;
        memcpy((char *)&legStateArray_, buffer_.data(), sizeof(RBQ_SDK::Motion::LegStateArray_t));
        buffer_.remove(0, buffer_.size());
        this->setLegStateArray(legStateArray_);
    } else {
        qDebug() << " -- ERROR -- buffer_.size() != sizeof(RBQ_SDK::RobotState_t)";
    }
}

static int reqFeedbackCount = 0;

void NetworkHandler::sendRequestMotionFeedback()
{
    if(nullptr == udpSocket) {
        return;
    }
    RBQ_SDK::Request_t request_;
    reqFeedbackCount++;
    if(reqFeedbackCount == 1) {
        request_.requestID = RBQ_SDK::Motion::RequestId_e::sendbackRobotState;
    } else if(reqFeedbackCount == 2) {
        request_.requestID = RBQ_SDK::Motion::RequestId_e::REQ_SENDBACK_LEG_STATE_ARRAY;
        reqFeedbackCount = 0;
    }
    QByteArray ba_ = QByteArray::fromRawData((const char *)&request_, sizeof(RBQ_SDK::Request_t));
    //    qDebug() << " -- sendRequestMotionFeedback:" << ba_.size() << " -- " <<
    udpSocket->writeDatagram(ba_);
}

void NetworkHandler::autoConnect()
{
    if(!m_connected) {
        qDebug() << " -- NetworkHandler::autoConnect() --";
        connect();
        return;
    }
}

void NetworkHandler::_sendTcpData(const QByteArray &msg)
{
    qDebug() << "NetworkHandler::sendTCP() motion TCP connected: " << m_connected;
    if(tcpSocket == nullptr) {
        return;
    }
    if(!m_connected) {
        qDebug() << "NetworkHandler::sendTCP() m_motionTCPconnected: " << m_connected;
        return;
    }
    tcpSocket->write(msg);
    tcpSocket->waitForBytesWritten(100);
}

void NetworkHandler::_sendUdpData(const QByteArray &msg)
{
    qDebug() << "NetworkHandler::sendUdp() motion TCP connected: " << m_connected;
    if(udpSocket == nullptr) {
        return;
    }
    if(!m_connected) {
        qDebug() << "NetworkHandler::sendUDP() m_motionTCPconnected: " << m_connected;
        return;
    }
    udpSocket->write(msg);
    udpSocket->waitForBytesWritten(100);
}
