#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace rbq_sdk {

struct JointTable {
    float ready[12] = {};
    float ground[12] = {};
    float folding[12] = {};

    JointTable() {
        for (int leg = 0; leg < 4; ++leg) {
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

struct JointControl {
    JointControl() {
        for (int i = 0; i < 12; ++i)
            m_joints[i] = std::make_unique<Joint>(i);
    }
    ~JointControl() = default;
    JointControl(const JointControl&) = delete;
    JointControl& operator=(const JointControl&) = delete;
    JointControl(JointControl&&) = default;
    JointControl& operator=(JointControl&&) = delete;

    enum ErrorCode {
        Ok = 0,
        GoalTimeInvalid,
        AlreadyMoving,
        ModeInvalid,
        SelectionInvalid
    };

    enum MovingStatus {
        Done = 0,
        InProgress
    };

    enum MoveCommandMode {
        Absolute = 0,
        Relative
    };

    double position(int idx) const {
        return m_joints[idx]->position();
    }

    void setPosition(const int &idx, const float &pos) {
        m_joints[idx]->setPosition(pos);
    }

    ErrorCode setMove(int idx, double angle, double duration, MoveCommandMode mode) {
        return m_joints[idx]->setTarget(angle, duration, mode);
    }

    MovingStatus update(int idx) {
        return m_joints.at(idx)->update();
    }

    void update() {
        for (int i = 0; i < 12; ++i)
            update(i);
    }

private:
    struct Joint {
        explicit Joint(int id = 0) : m_id(id) {}

        double position() const {
            return m_position;
        }

        void setPosition(const double &position) {
            m_isMoving = false;
            m_position = position;
        }

        ErrorCode setTarget(const double &position, double duration_ms, MoveCommandMode mode) {
            m_isMoving = false;
            if (duration_ms <= 0.0) {
                std::cerr << "[Joint " << m_id << "] Invalid goal time.\n";
                return ErrorCode::GoalTimeInvalid;
            }
            switch (mode) {
                case MoveCommandMode::Absolute:
                    m_targetPosition = position;
                    break;
                case MoveCommandMode::Relative:
                    m_targetPosition = m_position + position;
                    break;
                default:
                    std::cerr << "[Joint " << m_id << "] Invalid move mode.\n";
                    return ErrorCode::ModeInvalid;
            }
            m_initialPosition = m_position;
            m_deltaPosition = m_targetPosition - m_position;
            m_currentTimeCount = 0;
            m_goalTimeCount = static_cast<unsigned long>(duration_ms / 2);
            m_isMoving = true;
            return ErrorCode::Ok;
        }

        MovingStatus update() {
            if (!m_isMoving)
                return MovingStatus::InProgress;
            ++m_currentTimeCount;
            if (m_currentTimeCount >= m_goalTimeCount) {
                m_position = m_targetPosition;
                m_isMoving = false;
                return MovingStatus::Done;
            }
            const double progress = static_cast<double>(m_currentTimeCount) / m_goalTimeCount;
            m_position = m_initialPosition + m_deltaPosition * 0.5 * (1.0 - std::cos(M_PI * progress));
            return MovingStatus::InProgress;
        }

    private:
        int m_id;
        double m_position = 0;
        double m_initialPosition = 0;
        double m_targetPosition = 0;
        double m_deltaPosition = 0;
        unsigned long m_goalTimeCount = 0;
        unsigned long m_currentTimeCount = 0;
        bool m_isMoving = false;
    };
    std::array<std::unique_ptr<Joint>, 12> m_joints;
};

} // namespace rbq_sdk
