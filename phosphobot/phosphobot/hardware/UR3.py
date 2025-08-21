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
    # Теперь 6 UR3 + 1 actuated gripper joint (robotiq_85_left_knuckle_joint)
    SERVO_IDS: List[int] = [1, 2, 3, 4, 5, 6, 7]
    RESOLUTION: int = 4096

    # Reasonable default joint poses (radians)
    CALIBRATION_POSITION: List[float] = [0.0] * 7
    SLEEP_POSITION: List[float] | None = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0]

    # Defaults (will be refined at runtime from joint/link names)
    END_EFFECTOR_LINK_INDEX: int = 0
    GRIPPER_JOINT_INDEX: int = 11

    def __init__(self, only_simulation: bool = True, **kwargs):
        # Provide provisional indices to allow BaseManipulator init
        super().__init__(only_simulation=only_simulation, **kwargs)

        # After the URDF is loaded, refine indices based on names
        # We try to locate 'tool0' as end-effector link and
        # 'robotiq_85_left_knuckle_joint' as the gripper actuator
        try:
            if len(self.SERVO_IDS) != len(self.actuated_joints):
                self.SERVO_IDS = list(range(1, len(self.actuated_joints) + 1))
                logger.warning(f"UR3: SERVO_IDS updated to match actuated joints: {self.SERVO_IDS}")
            if len(self.CALIBRATION_POSITION) != len(self.SERVO_IDS):
                self.CALIBRATION_POSITION = [0.0] * len(self.SERVO_IDS)
                logger.warning(f"UR3: CALIBRATION_POSITION updated to match SERVO_IDS: {self.CALIBRATION_POSITION}")

            joint_count = len(self.lower_joint_limits)
            tool0_link_index: Optional[int] = None
            gripper_joint_index: Optional[int] = None

            for i in range(joint_count):
                info = self.sim.get_joint_info(self.p_robot_id, i)
                # info[1] -> jointName (bytes), info[12] -> linkName (bytes)
                try:
                    joint_name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
                    link_name = info[12].decode("utf-8") if isinstance(info[12], bytes) else str(info[12])
                except Exception:
                    joint_name = str(info[1])
                    link_name = str(info[12])

                if link_name == "tool0":
                    logger.warning(f"!!!!UR3: Found 'tool0' link at index {i}.")
                    tool0_link_index = i
                if joint_name == "robotiq_85_left_knuckle_joint":
                    logger.warning(f"!!!!UR3: Found Robotiq gripper joint at index {i}.")
                    gripper_joint_index = i

            if tool0_link_index is not None:
                self.END_EFFECTOR_LINK_INDEX = tool0_link_index
            else:
                logger.warning("UR3: Could not find 'tool0' link; using last wrist link for FK.")
                # Fallback to wrist_3 link (common for UR)
                self.END_EFFECTOR_LINK_INDEX = min(5, joint_count - 1)

            if gripper_joint_index is not None:
                self.GRIPPER_JOINT_INDEX = gripper_joint_index
            else:
                logger.warning(
                    "UR3: Could not find Robotiq gripper joint; using last revolute joint as gripper."
                )
                # Fallback to the last actuated joint
                self.GRIPPER_JOINT_INDEX = self.actuated_joints[-1]

            # Recompute initial angle for gripper for simulation control
            try:
                self.gripper_initial_angle = self.sim.get_joint_state(
                    robot_id=self.p_robot_id, joint_index=self.GRIPPER_JOINT_INDEX
                )[0]
            except Exception:
                pass

        except Exception as e:
            logger.warning(f"UR3: Post-init index refinement failed: {e}")

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

