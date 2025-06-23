#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rbq/Api.h"

// Constants
constexpr int kMaxJoint = 12;
constexpr int kMaxLeg = 4;
constexpr int kControlPeriodMs = 2;

// Enumerations
enum class ErrorCode {
    Ok = 0,
    GoalTimeInvalid,
    AlreadyMoving,
    ModeInvalid,
    SelectionInvalid
};

enum class MovingStatus {
    Done = 0,
    InProgress
};

enum class MoveCommandMode {
    Absolute = 0,
    Relative
};

// Joint position presets
struct JointTable {
    float ready[kMaxJoint] = {};
    float ground[kMaxJoint] = {};
    float folding[kMaxJoint] = {};

    JointTable() {
        for (int leg = 0; leg < kMaxLeg; ++leg) {
            int offset = leg * 3;
            ready[offset] = 0.0f;
            ready[offset + 1] = 43.0f;
            ready[offset + 2] = -80.0f;

            ground[offset] = (leg % 2 == 0) ? -34.0f : 34.0f;
            ground[offset + 1] = 65.0f;
            ground[offset + 2] = -160.0f;

            folding[offset] = 0.0f;
            folding[offset + 1] = 78.0f;
            folding[offset + 2] = -160.0f;
        }
    }
};

class Joint {
public:
    explicit Joint(int id = 0)
        : m_id(id), m_currentAngle(0.0), m_initialAngle(0.0), m_targetAngle(0.0),
          m_deltaAngle(0.0), m_goalTimeCount(0), m_currentTimeCount(0), m_isMoving(false) {}

    void setCurrentAngle(double angle) { m_currentAngle = angle; }
    double getCurrentAngle() const { return m_currentAngle; }
    void setMoving(bool flag) { m_isMoving = flag; }

    ErrorCode setTarget(double angle, double duration_ms, MoveCommandMode mode) {
        if (duration_ms <= 0.0) {
            std::cerr << "[Joint " << m_id << "] Invalid goal time.\n";
            return ErrorCode::GoalTimeInvalid;
        }

        m_isMoving = false;
        switch (mode) {
            case MoveCommandMode::Absolute:
                m_targetAngle = angle;
                break;
            case MoveCommandMode::Relative:
                m_targetAngle = m_currentAngle + angle;
                break;
            default:
                std::cerr << "[Joint " << m_id << "] Invalid move mode.\n";
                return ErrorCode::ModeInvalid;
        }

        m_initialAngle = m_currentAngle;
        m_deltaAngle = m_targetAngle - m_currentAngle;
        m_currentTimeCount = 0;
        m_goalTimeCount = static_cast<unsigned long>(duration_ms / kControlPeriodMs);
        m_isMoving = true;

        return ErrorCode::Ok;
    }

    MovingStatus update() {
        if (!m_isMoving) return MovingStatus::InProgress;

        ++m_currentTimeCount;
        if (m_currentTimeCount >= m_goalTimeCount) {
            m_currentAngle = m_targetAngle;
            m_isMoving = false;
            return MovingStatus::Done;
        }

        double progress = static_cast<double>(m_currentTimeCount) / m_goalTimeCount;
        m_currentAngle = m_initialAngle + m_deltaAngle * 0.5 * (1.0 - std::cos(M_PI * progress));
        return MovingStatus::InProgress;
    }

private:
    int m_id;
    double m_currentAngle, m_initialAngle, m_targetAngle, m_deltaAngle;
    unsigned long m_goalTimeCount, m_currentTimeCount;
    bool m_isMoving;
};

class JointController {
public:
    JointController(RBQ_API* api, int joint_count)
        : m_api(api), m_jointCount(joint_count) {

        if (!m_api) throw std::invalid_argument("RBQ_API pointer is null.");
        if (joint_count <= 0 || joint_count > kMaxJoint)
            throw std::out_of_range("Invalid joint count.");

        for (int i = 0; i < m_jointCount; ++i)
            m_joints[i] = std::make_unique<Joint>(i);
    }

    ~JointController() = default;

    JointController(const JointController&) = delete;
    JointController& operator=(const JointController&) = delete;
    JointController(JointController&&) = default;
    JointController& operator=(JointController&&) = default;

    double getAngle(int idx) const { return m_joints[idx]->getCurrentAngle(); }
    void setAngle(int idx, double angle) { m_joints[idx]->setCurrentAngle(angle); }

    void setOwner(int idx) { m_api->Joint.setMotionOwner(idx); }
    void setAllOwners() {
        for (int i = 0; i < m_jointCount; ++i)
            setOwner(i);
    }

    ErrorCode moveJoint(int idx, double angle, double duration, MoveCommandMode mode) {
        return m_joints[idx]->setTarget(angle, duration, mode);
    }

    MovingStatus updateJoint(int idx) {
        return m_joints.at(idx)->update();
    }

    void updateAllJoints() {
        for (int i = 0; i < m_jointCount; ++i)
            updateJoint(i);
    }

    void syncReferenceToRobot() {
        for (int i = 0; i < m_jointCount; ++i)
            syncReferenceToRobot(i);
    }

    void syncPositionToRobot() {
        for (int i = 0; i < m_jointCount; ++i)
            syncPositionToRobot(i);
    }

    void syncReferenceToRobot(int idx) {
        float ref;
        m_api->Joint.getPosRef(idx, ref);
        m_api->Joint.setPosRef(idx, ref);
        m_joints[idx]->setMoving(false);
        m_joints[idx]->setCurrentAngle(ref);
    }

    void syncPositionToRobot(int idx) {
        float pos;
        m_api->Joint.getPos(idx, pos);
        m_api->Joint.setPosRef(idx, pos);
        m_joints[idx]->setMoving(false);
        m_joints[idx]->setCurrentAngle(pos);
    }

    void sendReferencesToRobot() {
        for (int i = 0; i < m_jointCount; ++i)
            m_api->Joint.setPosRef(i, m_joints[i]->getCurrentAngle());
    }

private:
    const int m_jointCount;
    RBQ_API* m_api;
    std::array<std::unique_ptr<Joint>, kMaxJoint> m_joints;
};

#endif  // JOINT_CONTROL_H
