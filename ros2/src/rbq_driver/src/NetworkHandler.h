#ifndef NETWORKHANDLER_H
#define NETWORKHANDLER_H

#include <QObject>

#include "RBTypes.hpp"

class NetworkHandler : public QObject
{
    Q_OBJECT

public:
    explicit NetworkHandler(QObject *parent = nullptr, const std::string &host = "192.168.0.10", const int &_commFrequency = 100);
    ~NetworkHandler();

    void connect();
    void disconnect();

    void sendTCP(const QByteArray &msg);
    void sendUDP(const QByteArray &msg);
    int motionFeedbackFrequency() const { return m_motionFeedbackFrequency; }
    void setMotionFeedbackFrequency(const int &newFrequency = 1);

    void setRobotState(const RBQ_SDK::Motion::RobotState_t &newRobotState)
    {
        if(nullptr == m_robotState) {
            m_robotState = new RBQ_SDK::Motion::RobotState_t;
        }
        memcpy((char *)m_robotState, (char *)&newRobotState, sizeof(RBQ_SDK::Motion::RobotState_t));
        Q_EMIT robotStateChanged();
    }
    RBQ_SDK::Motion::RobotState_t *robotState() const { return m_robotState; }

    void setLegStateArray(const RBQ_SDK::Motion::LegStateArray_t &newLegStateArray)
    {
        if(nullptr == m_legStateArray) {
            m_legStateArray = new RBQ_SDK::Motion::LegStateArray_t;
        }
        memcpy((char *)m_legStateArray, (char *)&newLegStateArray, sizeof(RBQ_SDK::Motion::LegStateArray_t));
        Q_EMIT legStateArrayChanged();
    }
    RBQ_SDK::Motion::LegStateArray_t *legStateArray() const { return m_legStateArray; }

private Q_SLOTS:
    void onConnected() { m_connected = true; }
    void onDisconnected() { m_connected = false; }
    void readSocketMotionDatagram();
    void sendRequestMotionFeedback();
    void autoConnect();

    void _sendTcpData(const QByteArray &msg);
    void _sendUdpData(const QByteArray &msg);

Q_SIGNALS:
    void robotStateChanged();
    void legStateArrayChanged();
    void sendTcpData(const QByteArray &msg);
    void sendUdpData(const QByteArray &msg);

private:
    bool m_connected = false;
    int m_motionFeedbackFrequency = 100;
    RBQ_SDK::Motion::RobotState_t* m_robotState = nullptr;
    RBQ_SDK::Motion::LegStateArray_t* m_legStateArray = nullptr;
};

#endif // NETWORKHANDLER_H
