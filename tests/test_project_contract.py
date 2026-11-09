"""Offline migration checks using real ROS message types, without a master."""
import importlib.util
import importlib.machinery
from pathlib import Path
import pickle
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "app"))
import plan
import replay
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

loader = importlib.machinery.SourceFileLoader("robotctl", str(ROOT / "robotctl"))
spec = importlib.util.spec_from_loader(loader.name, loader)
ctl = importlib.util.module_from_spec(spec)
loader.exec_module(ctl)


class ProjectContractTest(unittest.TestCase):
    def test_original_stage_sequence_is_retained(self):
        self.assertEqual(replay.SEQUENCE, [
            ("hand", "ZERO"),
            ("hand", "Stage_1_Hand_thumb-only.pkl"),
            ("arm", "Stage_2_Move_arm_to_captured_hand_pose.pkl"),
            ("hand", "Stage_3_Move_all_fingers.pkl"),
            ("arm", "Stage_4_Move_arm_with_palm_shift.pkl"),
            ("hand", "Stage_4_Iterated_hand_joint_angle_changes.pkl"),
            ("arm", "Stage_5a_Move_arm_to_additional_offset_pose.pkl"),
            ("arm", "Stage_5_Straight-line_Z_lift.pkl"),
            ("arm", "After_Stage_5_Planned_joint_move_to_home.pkl"),
        ])

    def test_saved_ros_trajectory_and_delta_formats_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            arm, hand = Path(folder, "arm"), Path(folder, "hand")
            with patch.object(plan, "ARM_TRAJ_DIR", str(arm)), patch.object(plan, "HAND_TRAJ_DIR", str(hand)):
                trajectory = RobotTrajectory()
                trajectory.joint_trajectory.joint_names = ["ra_elbow_joint"]
                trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.5])]
                plan.save_traj(trajectory, "Stage 2/5: Move arm to captured hand pose", "arm")
                path = arm / replay.SEQUENCE[2][1]
                self.assertEqual(pickle.loads(path.read_bytes()), trajectory)
                delta = {"rh_FFJ1": {"delta": 0.1}}
                plan.save_joint_delta(delta, "Stage 4/5: Iterated hand joint angle changes")
                self.assertEqual(pickle.loads((hand / replay.SEQUENCE[5][1]).read_bytes()), delta)
                plan.save_hand_target_joints({"rh_THJ1": 0.2}, "Stage 1/5: Hand thumb-only")
                self.assertEqual(pickle.loads((hand / "Stage_1_Hand_thumb-only_target_joints.pkl").read_bytes()), {"rh_THJ1": 0.2})

    def test_both_modes_invoke_one_shared_replay(self):
        cfg = ctl.configuration()
        with patch.object(ctl, "master"), patch.object(ctl, "ros_command") as execute:
            for mode in ("sim", "real"):
                for command, flag in (("check", "--check"), ("replay", "--step"), ("run", "--run")):
                    ctl.mode_command(cfg, mode, command, [])
                    self.assertEqual(execute.call_args.args[2], ["python3", "/project/app/replay.py", flag])

    def test_arm_pose_execution_failure_stops_planning_workflow(self):
        group = Mock()
        group.execute.return_value = False
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0])]
        with patch.object(plan, "plan", return_value=trajectory), patch.object(plan, "save_traj"):
            with self.assertRaisesRegex(RuntimeError, "execute failed"):
                plan.execute_pose(group, plan.ARM_POSE_STAGE2, "Stage 2")
        group.stop.assert_called_once()

    def test_missing_master_blocks_nuc_before_ssh(self):
        with patch.object(ctl, "master", side_effect=ctl.UserError("offline")), patch.object(ctl, "ssh") as ssh:
            with self.assertRaises(ctl.UserError):
                ctl.start_nuc(ctl.configuration())
            ssh.assert_not_called()

    def test_failed_calibration_does_not_stop_remote_container(self):
        with patch.object(ctl, "master"), patch.object(ctl, "ros_command", side_effect=subprocess.CalledProcessError(1, "calibration")), patch.object(ctl, "ssh") as ssh:
            with self.assertRaises(subprocess.CalledProcessError):
                ctl.shutdown_real(ctl.configuration())
            ssh.assert_not_called()

    def test_modes_cannot_be_started_together(self):
        with patch.object(ctl, "compose", return_value=Mock(stdout="workstation\nur5\n")) as compose:
            with self.assertRaises(ctl.UserError):
                ctl.mode_command(ctl.configuration(), "sim", "up", [])
            self.assertEqual(compose.call_count, 1)


if __name__ == "__main__":
    unittest.main()
