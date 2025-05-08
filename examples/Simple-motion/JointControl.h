#ifndef JOINT_CONTROL_H
#define JOINT_CONTROL_H

#include <iostream>
#include <math.h>

#include "rbq/Api.h"

// enum variables
enum ErrCode{
    ERR_OK = 0,
    ERR_GOAL_TIME,
    ERR_ALREADY_MOVING,
    ERR_WRONG_MODE,
    ERR_WRONG_SELECTION
};
enum MovingStatus{
    MOVE_DONE = 0,
    STILL_MOVING
};
enum MoveCommandMode{
    MOVE_ABSOLUTE = 0,
    MOVE_RELATIVE,
};

#define MAX_JOINT   12
#define MAX_LEG     4

#define RT_MS       2

struct JOINT_TABLE
{
    //--- Home joint set
    float   JointReady[MAX_JOINT];
    float   JointGround[MAX_JOINT];
    float   JointFolding[MAX_JOINT];

    JOINT_TABLE() {
        for(int lnum=0; lnum<MAX_LEG; lnum++){

            //ready
            JointReady[lnum*3] = 0;
            JointReady[lnum*3+1] = 43.0;
            JointReady[lnum*3+2] = -(80.0);
            //ground
            if(lnum%2 == 0)
                JointGround[lnum*3] = -34.0;
            else
                JointGround[lnum*3] = 34.0;
            JointGround[lnum*3+1] = 65.0;
            JointGround[lnum*3+2] = -160.0;
            //falling
            JointFolding[lnum*3] = 0;
            JointFolding[lnum*3+1] = 78.0;
            JointFolding[lnum*3+2] = -160.0;
        }
    }
};

class Joint
{
public:
    int         JNum;
    double      RefAngleCurrent;

public:
    Joint(){
        RefAngleCurrent = 0.f; MoveFlag = false;
        RefAngleInitial = 0; RefAngleToGo = 0;
    }
    Joint(const int jnum){
        JNum = jnum; RefAngleCurrent = 0.f; MoveFlag = false;
        RefAngleInitial = 0; RefAngleToGo = 0;
    }

    void    SetRefAngleCurrent(const double ref)	{RefAngleCurrent = ref;}
    double  GetRefAngleCurrent()					{return RefAngleCurrent;}
    void    SetMoveFlag(unsigned char flag)         {MoveFlag = flag;}

    char    SetMoveJoint(const double _angle, const double _msTime, const unsigned int _mode){
        if(_msTime <= 0){
            std::cout << "Goal time must be greater than zero(SetMoveJoint)[JNum: " << JNum << "]"; return ERR_GOAL_TIME;
        }

        MoveFlag = false;
        switch(_mode)
        {
        case MOVE_ABSOLUTE:	// absolute mode
            RefAngleToGo = _angle;
            break;
        case MOVE_RELATIVE:	// relative mode
            RefAngleToGo = RefAngleCurrent + _angle;
            break;
        default:
            std::cout << "Wrong reference mode(SetMoveJoint)[JNum: " << JNum << "]";
            return ERR_WRONG_MODE;
            break;
        }
        RefAngleInitial = RefAngleCurrent;
        RefAngleDelta = RefAngleToGo - RefAngleCurrent;
        CurrentTimeCount = 0;

        GoalTimeCount = (unsigned long)(_msTime/RT_MS);
        MoveFlag = true;
        return ERR_OK;
    }
    char    MoveJoint(){
        if(MoveFlag == true){
            CurrentTimeCount++;
            if(GoalTimeCount <= CurrentTimeCount){
                GoalTimeCount = CurrentTimeCount = 0;
                RefAngleCurrent = RefAngleToGo;
                MoveFlag = false;
                return MOVE_DONE;
            }else{
                RefAngleCurrent = RefAngleInitial+RefAngleDelta*0.5f*(1.0f-cos(M_PI/(double)GoalTimeCount*(double)CurrentTimeCount));
            }
        }
        return STILL_MOVING;
    }

private:
    double			RefAngleDelta;
    double			RefAngleToGo;
    double			RefAngleInitial;
    unsigned long	GoalTimeCount;
    unsigned long	CurrentTimeCount;
    unsigned char	MoveFlag;
};

class JointControl
{
public:
    explicit JointControl(RBQ_API *_rbqApi, int _joint_num)
        : m_api{_rbqApi}
        , NO_OF_JOINT{_joint_num}
    {
        for(int i=0; i<NO_OF_JOINT; i++)
            m_joints.push_back(new Joint(i));
    }

    double GetJointRefAngle(const int n) {return m_joints[n]->GetRefAngleCurrent();}
    void SetJointRefAngle(const int n, const double _ref) {m_joints[n]->SetRefAngleCurrent(_ref);}

    void SetMotionOwner(const int &_jnum){
        m_api->Joint.setMotionOwner(_jnum);
    }
    void SetAllMotionOwner(){
        for(int i=0; i<NO_OF_JOINT; i++){
            SetMotionOwner(i);
        }
    }

    char SetMoveJoint(const int _jnum, const double _angle, const double _msTime, const unsigned int _mode){
        return m_joints[_jnum]->SetMoveJoint(_angle, _msTime, _mode);
    }
    char MoveJoint(const int _jnum){
        return m_joints[_jnum]->MoveJoint();
    }
    void MoveAllJoint(){
        for(int i=0; i<NO_OF_JOINT; i++){
            MoveJoint(i);
        }
    }

    void JointUpdate(){
        for(int i=0; i<NO_OF_JOINT; i++){
            m_api->Joint.setPosRef(i, m_joints[i]->GetRefAngleCurrent());
        }
    }

    void RefreshToCurrentReference_id(int &_jointId){
        // Refresh internal joint reference variable to current robot joint reference

        m_joints[_jointId]->SetMoveFlag(false);

        float current_joint_pos_reference_of_robot;
        m_api->Joint.getPosRef(_jointId, current_joint_pos_reference_of_robot);
        m_api->Joint.setPosRef(_jointId, current_joint_pos_reference_of_robot);

        m_joints[_jointId]->SetRefAngleCurrent(current_joint_pos_reference_of_robot);
    }

    void RefreshToCurrentPosition_id(int &_jointId){
        // Refresh internal joint reference variable to current robot joint measured

        m_joints[_jointId]->SetMoveFlag(false);

        float current_joint_pos_measured_of_robot;
        m_api->Joint.getPos(_jointId, current_joint_pos_measured_of_robot);
        m_api->Joint.setPosRef(_jointId, current_joint_pos_measured_of_robot);

        m_joints[_jointId]->SetRefAngleCurrent(current_joint_pos_measured_of_robot);
    }

    void RefreshToCurrentReference()
    {
        // Refresh internal joint reference variable to current robot joint reference
        for(int i=0; i<NO_OF_JOINT; i++){
            RefreshToCurrentReference_id(i);
        }
    }

    void RefreshToCurrentPosition()
    {
        // Refresh internal joint reference variable to current robot joint measured

        for(int i=0; i<NO_OF_JOINT; i++){
            RefreshToCurrentPosition_id(i);
        }
    }

private:
    int                     NO_OF_JOINT;
    RBQ_API                 *m_api;
    std::vector<Joint*>     m_joints;
};

#endif // JOINT_CONTROL_H
