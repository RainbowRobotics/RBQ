
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from rbq_lab import LAB_ASSET_DIR

##
# Configuration
##

D2R = 3.14159265358979323846 / 180.0  
R2D = 180.0 / 3.14159265358979323846 

RBQ10_LEG_JOINT = [
            'joint0_HRR' ,
            'joint1_HRP' ,
            'joint2_HRK' ,
            'joint3_HLR' ,
            'joint4_HLP' ,
            'joint5_HLK' ,
            'joint6_FRR' ,
            'joint7_FRP' ,
            'joint8_FRK' ,
            'joint9_FLR' ,
            'joint10_FLP',
            'joint11_FLK',
]

RBQ10_HIP_JOINT = [
            'joint0_HRR' ,
            'joint3_HLR' ,
            'joint6_FRR' ,
            'joint9_FLR' ,
]

RBQ10_THIGH_JOINT = [
            'joint1_HRP' ,
            'joint4_HLP' ,
            'joint7_FRP' ,
            'joint10_FLP' ,
]

RBQ10_KNEE_JOINT = [
            'joint2_HRK' ,
            'joint5_HLK' ,
            'joint8_FRK' ,
            'joint11_FLK' ,
]


RBQ10_URDF_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        # asset_path="/home/jsh/Quadruped/RBQ_ws/RBQ/main/resources/urdf/rbq10.urdf",
        asset_path=f'{LAB_ASSET_DIR}/urdf/rbq10.urdf',
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, 
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=0,
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.01, 
            rest_offset=0.0,
        ),        
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),

    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.55),
        joint_pos={ # = target angles [rad] when action = 0.0
            'joint0_HRR': -0.,
            'joint1_HRP': 46.5*D2R,
            'joint2_HRK': -80*D2R,
            'joint3_HLR': 0.,
            'joint4_HLP': 46.5*D2R,
            'joint5_HLK': -80*D2R,
            'joint6_FRR': -0.,
            'joint7_FRP': 45.5*D2R,
            'joint8_FRK': -82*D2R,
            'joint9_FLR': 0.,
            'joint10_FLP': 45.5*D2R,
            'joint11_FLK': -82*D2R,
        },
    ),

    actuators={
        "legs_R":ImplicitActuatorCfg(
            joint_names_expr=[".*HRR", ".*HLR", ".*FRR", ".*FLR"],
            stiffness=123.39138,       # [N*m/rad]
            damping=2.5,               # [N*m*s/rad]
            velocity_limit=50,       # [rad/s]
            effort_limit=100,         # [N*m] continuous
            friction=0.0,
            armature=0.014058,      # [kg*m^2]
        ),
        "legs_P": ImplicitActuatorCfg(
            joint_names_expr=[".*HRP", ".*HLP", ".*FRP", ".*FLP"],
            stiffness=123.39138,        
            damping=2.5,
            velocity_limit=50,
            effort_limit=100,
            friction=0.0,
            armature=0.014058,
        ),
        "legs_K": ImplicitActuatorCfg(
            joint_names_expr=[".*HRK", ".*HLK", ".*FRK", ".*FLK"],
            stiffness=127.77215,
            damping=2.5,
            velocity_limit=50,
            effort_limit=150,
            friction=0.0,
            armature=0.0214816,
        ),
    },
    soft_joint_pos_limit_factor=0.9,
)