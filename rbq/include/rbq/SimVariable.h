#ifndef SIMVARIABLE_H
#define SIMVARIABLE_H

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#define SIM_DATA       "SIMULATION_DATA"

#define MAX_MANI 6
#define MAX_GRP  1

typedef struct _SIM_VARIABLE_
{
    ///-----------------------------------Simulator variables ------------------------------------------
    bool   TimeSyncFlag;
    float  Joint_pGain[12];
    float  Joint_dGain[12];
    float  choreonoid_gyro[3], choreonoid_acc[3];
    float  choreonoid_quat[4];

    float choreonoid_JointPosNow[12];
    float choreonoid_JointVelNow[12];

    float choreonoid_refJointPos[12];
    float choreonoid_refJointVel[12];
    float choreonoid_refJointTorque[12];

    float chorednoid_ReactionF_HR[3], chorednoid_ReactionF_HL[3],
            chorednoid_ReactionF_FR[3], chorednoid_ReactionF_FL[3];

    float  choreonoid_W_body_pos[3];
    
    
    ///-------------------------------Manipulator---------------------------------------------
    bool    is_arm;
    float   Joint_pGain_Mani[MAX_MANI+MAX_GRP];
    float   Joint_dGain_Mani[MAX_MANI+MAX_GRP];
    float   choreonoid_JointPosNow_Mani[MAX_MANI+MAX_GRP];
    float   choreonoid_JointVelNow_Mani[MAX_MANI+MAX_GRP];
    float   choreonoid_refJointPos_Mani[MAX_MANI+MAX_GRP];
    float   choreonoid_refJointVel_Mani[MAX_MANI+MAX_GRP];
    float   choreonoid_refJointTorque_Mani[MAX_MANI+MAX_GRP];
    
    ///------------------------------Marker position---------------------------------------------------
    float pos_zmpRef[3], pos_comRef[3];
    float pos_pfoot0[3], pos_pfoot1[3], pos_pfoot2[3], pos_pfoot3[3];
    bool CommandRef_contact[4], State_contact[4];
    float CommandRef_swingState[4];

    ///-----------------------------Contact Estimattion------------------------------------------
    bool Landing_Estimation_flag[4], SwingLeg_Collision_falg[4];

}SIM_VARIABLE, *pSIM_VARIABLE;

#endif // SIMVARIABLE_H
