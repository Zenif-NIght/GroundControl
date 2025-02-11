# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# Copyright (c) 2022-2025, The GroundControl Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from isaaclab.sensors.camera.camera_cfg import PinholeCameraCfg
#from isaaclab.sensors.camera import TiledCameraCfg

# ===== NOTE:IsaacLab imports === ^^^ 
# ===== GroundControl imports === VVV
from groundcontrol import GROUNDCONTROL_EXT_DIR
import groundcontrol_tasks.manager_based.navigation.mdp as mdp
from groundcontrol_tasks.manager_based.locomotion.velocity.config.spot.flat_env_cfg import SpotFlatEnvCfg
from groundcontrol.sensors.camera import TiledCameraCfg #TODO: migrate to IsaacLab cameras when ready

LOW_LEVEL_ENV_CFG = SpotFlatEnvCfg()

@configclass
class EventCfg:
    """Configuration for events."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.0, 0.0),
                "y": (-0.0, 0.0),
                "z": (-0.0, 0.0),
                "roll": (-0.0, 0.0),
                "pitch": (-0.0, 0.0),
                "yaw": (-0.0, 0.0),
            },
        },
    )

@configclass
class ActionsCfg:
    """Action terms for the MDP."""

    pre_trained_policy_action: mdp.PreTrainedPolicyActionCfg = mdp.PreTrainedPolicyActionCfg(
        asset_name="robot",
        policy_path="UWRobotics/SpotBlindFlat",
        low_level_decimation=4,
        low_level_actions=LOW_LEVEL_ENV_CFG.actions.joint_pos,
        low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class HighlevelPolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        projected_gravity = ObsTerm(func=mdp.projected_gravity)

        pose_command = ObsTerm(func=mdp.generated_commands, params={"command_name": "pose_command"})

    @configclass
    class PerceptionCfg(ObsGroup):
        concatenate_terms = False
        # rgb_camera = ObsTerm(
        #     func=mdp.isaac_camera_data,
        #     params={"sensor_cfg": SceneEntityCfg("rgb_camera"), "data_type": "rgb"},
        # )
        #rgb_camera2 = ObsTerm(
        #    func=mdp.isaac_camera_data,
        #    params={"sensor_cfg": SceneEntityCfg("rgb_camera"), "data_type": "rgb"},
        #)
        # TODO: add more cameras later
        # TODO: note that cameras are not within HighLevelPolicyCfg, this is because the policy does not depend on camera atm 
        # TODO: these cameras do not need to be in the ObservationCfg for them to be accessed in the GUI, the only need to be in self.scene
    # observation groups
    policy: HighlevelPolicyCfg = HighlevelPolicyCfg()
    perception: PerceptionCfg = PerceptionCfg()


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-400.0)

    position_tracking = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.5,
        params={"std": 2.0, "command_name": "pose_command"},
    )
    position_tracking_fine_grained = RewTerm(
        func=mdp.position_command_error_tanh,
        weight=0.5,
        params={"std": 0.2, "command_name": "pose_command"},
    )
    orientation_tracking = RewTerm(
        func=mdp.heading_command_error_abs,
        weight=-0.2,
        params={"command_name": "pose_command"},
    )


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    pose_command = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(8.0, 8.0),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(pos_x=(-3.0, 3.0), pos_y=(-3.0, 3.0), heading=(-math.pi, math.pi)),
    )

    # Example of setting a goal pose
    goal_pose = mdp.UniformPose2dCommandCfg(
        asset_name="robot",
        simple_heading=False,
        resampling_time_range=(1000.0, 1000.0),
        debug_vis=True,
        ranges=mdp.UniformPose2dCommandCfg.Ranges(pos_x=(5.0, 5.0), pos_y=(-5.0, -5.0), heading=(math.pi, math.pi)),
    )


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    pass


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="body"), "threshold": 1.0},
    )
#Note: For Spot, the body name is "body" rather than "base".

@configclass
class NavigationEnvCfg(SpotFlatEnvCfg):
    # Note: We need to override SceneEntityCfg to have the correct fields to add the camera
    #scene: SceneEntityCfg = NavigationSceneEntityCfg()
    scene: SceneEntityCfg = LOW_LEVEL_ENV_CFG.scene
    commands: CommandsCfg = CommandsCfg()
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    events: EventCfg = EventCfg()

    curriculum: CurriculumCfg = CurriculumCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        self.sim.dt = LOW_LEVEL_ENV_CFG.sim.dt
        self.sim.render_interval = LOW_LEVEL_ENV_CFG.decimation
        self.decimation = LOW_LEVEL_ENV_CFG.decimation * 10
        
        self.episode_length_s = self.commands.pose_command.resampling_time_range[1]

        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = (
                self.actions.pre_trained_policy_action.low_level_decimation * self.sim.dt
            )
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        # NOTE: for the moment, the camera is added in the post_init of NavigationEnvCfg
        # We can create a MySceneEntityCfg class for the NavigationEnvCfg as well instead
        self.scene.rgb_camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/body/base_cam",
            update_period=0.0,
            height=64,
            width=64,
            data_types=["rgb"],
            spawn=PinholeCameraCfg(),
            offset=TiledCameraCfg.OffsetCfg(
                pos=(-0.2, 0, 0.2),
                rot=(0.5, -0.5, 0.5, -0.5),
            ),
        )

@configclass
class NavigationEnvCfg_PLAY(NavigationEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False

        # Episode Termination Length
        self.episode_length_s = 20.0

        from isaaclab.assets import AssetBaseCfg
        import isaaclab.sim as sim_utils

        self.scene.terrain = AssetBaseCfg(
            prim_path="/World/Ground",
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(0,0,-5)
            ),
            spawn=sim_utils.UsdFileCfg(
                usd_path="/home/simdev/WRK/00_USD/Collected_GQ_lite/GQ_lite.usd",
                scale=(0.01, 0.01, 0.01),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    kinematic_enabled=True,
                ),
                # collision_props=sim_utils.CollisionPropertiesCfg()
            ),
        )
