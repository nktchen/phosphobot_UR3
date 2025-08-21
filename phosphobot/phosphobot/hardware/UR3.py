from typing import List, Optional

import numpy as np
from loguru import logger

from phosphobot.hardware.base import BaseManipulator
from phosphobot.utils import get_resources_path


class UR3Hardware(BaseManipulator):
    name: str = "ur3"

    # URDF and Configuration
    URDF_FILE_PATH = str(
        get_resources_path() / "urdf" / "UR3" / "urdf" / "UR3_with_gripper_bullet.urdf"
    )

    # Axis orientation (quaternion) and base placement
    AXIS_ORIENTATION = [0, 0, 0, 1]

    # Logical servo identifiers (simulation-only; used for shapes/lengths)
    # 6 UR3 + 6 actuated gripper joint
    # shoulder_pan_joint, 'shoulder_lift_joint', elbow_joint, 
    # 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint' 
    # robotiq_85_left_knuckle_joint, robotiq_85_right_knuckle_joint, 
    # robotiq_85_left_inner_knuckle_joint, robotiq_85_right_inner_knuckle_joint
    # robotiq_85_left_finger_tip_joint, robotiq_85_right_finger_tip_joint
    SERVO_IDS: List[int] = [0, 1, 2, 3, 4, 5, 10, 11, 14, 15, 16, 17]
    RESOLUTION: int = 4096

    CALIBRATION_POSITION: List[float] = [0.0] * 12
    SLEEP_POSITION: List[float] | None = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0] + [0.0] * 6

    END_EFFECTOR_LINK_INDEX: int = 7
    GRIPPER_JOINT_INDEX: int = 10

    def __init__(self, only_simulation: bool = True, **kwargs):
        # Provide provisional indices to allow BaseManipulator init
        super().__init__(only_simulation=only_simulation, **kwargs)
        # Recompute initial angle for gripper for simulation control
        self.gripper_initial_angle = self.sim.get_joint_state(
            robot_id=self.p_robot_id, joint_index=self.GRIPPER_JOINT_INDEX
        )[0]
        
    # Simulation-only driver: no real hardware IO below
    async def connect(self) -> None:
        self.is_connected = False
        self.init_config()

    def disconnect(self) -> None:
        self.is_connected = False

    def enable_torque(self) -> None:
        # No-op in pure simulation
        return

    def disable_torque(self) -> None:
        # No-op in pure simulation
        return

    def read_motor_torque(self, servo_id: int) -> float | None:
        # Not available without hardware
        return None

    def read_motor_voltage(self, servo_id: int) -> float | None:
        # Not available without hardware
        return None

    def write_motor_position(self, servo_id: int, units: int, **kwargs) -> None:
        # Not available without hardware
        return

    def read_motor_position(self, servo_id: int, **kwargs) -> int | None:
        # Not available without hardware
        return None

    def calibrate_motors(self, **kwargs) -> None:
        # Not applicable in simulation-only mode
        return

    def write_group_motor_position(self, q_target: np.ndarray, enable_gripper: bool) -> None:
        # Not available without hardware; simulation is handled in BaseManipulator.set_motors_positions
        return

    def read_group_motor_position(self) -> np.ndarray:
        # Not available without hardware
        return np.ones(len(self.SERVO_IDS)) * np.nan

    def get_default_base_robot_config(self, voltage: str, raise_if_none: bool = False):
        # Provide a generic config for simulation
        from phosphobot.models import BaseRobotConfig, BaseRobotPIDGains

        n = len(self.SERVO_IDS)
        pid = [BaseRobotPIDGains(p_gain=12, i_gain=0, d_gain=36) for _ in range(n)]
        # Ensure offsets and calibration positions are different to satisfy validator
        servos_offsets = [0.0] * n
        servos_calibration_position = [0.1] * n
        servos_offsets_signs = [1.0] * n
        return BaseRobotConfig(
            name=self.name,
            servos_voltage=12.0,
            servos_offsets=servos_offsets,
            servos_offsets_signs=servos_offsets_signs,
            servos_calibration_position=servos_calibration_position,
            pid_gains=pid,
        )

