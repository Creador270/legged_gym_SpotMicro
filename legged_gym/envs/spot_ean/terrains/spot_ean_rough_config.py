from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class SpotEANRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env ):
        num_envs = 4096
        num_actions = 12
    
    class terrain( LeggedRobotCfg.terrain ):
        mesh_type = 'trimesh'
        
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.2] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'joint_front_left_shoulder': 0.0,  # [rad]
            'joint_front_right_shoulder': 0.0, # [rad]
            'joint_rear_left_shoulder': 0.0, # [rad]
            'joint_rear_right_shoulder': 0.0, # [rad]

            'joint_front_left_leg': -0.891,   # [rad]
            'joint_front_right_leg': -0.891,   # [rad]
            'joint_rear_left_leg': -0.891,   # [rad]
            'joint_rear_right_leg': -0.891,   # [rad]

            'joint_front_left_foot': 1.834,   # [rad]
            'joint_front_right_foot': 1.834,   # [rad]
            'joint_rear_left_foot': 1.834,   # [rad]
            'joint_rear_right_foot': 1.834,   # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 80.}  # [N*m/rad]
        damping = {'joint': 2}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4
        use_actuator_network = True
        actuator_net_file = "{LEGGED_GYM_ROOT_DIR}/resources/actuator_nets/anydrive_v3_lstm.pt"


    class asset( LeggedRobotCfg.asset ):
        file = "{LEGGED_GYM_ROOT_DIR}/resources/robots/spot_ean/urdf/spotmicroaiean.urdf"
        name = "SpotMicroAI"
        foot_name = "feet"
        penalize_contacts_on = ["leg", "foot"] 
        terminate_after_contacts_on = ["base_link"]
        flip_visual_attachments = False
        self_collisions = 0 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.2
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -5.0

class SpotEANRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_spot_ean'

  