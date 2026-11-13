import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from sim_contacts import contact_matrix, hand_object_matrix, configure_hand_object_contacts
import sim_contacts


class ContactTest(unittest.TestCase):
    def test_replay_allows_self_world_and_attached_contacts_then_restores_exact_matrix(self):
        from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
        scene = PlanningScene()
        scene.allowed_collision_matrix = AllowedCollisionMatrix(
            entry_names=['rh_palm', 'floor'],
            entry_values=[AllowedCollisionEntry(enabled=[False, False]) for _ in range(2)],
            default_entry_names=['floor'], default_entry_values=[False])
        scene.world.collision_objects = [CollisionObject(id='rect_object')]
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(object=CollisionObject(id='held'))]
        original = copy.deepcopy(scene.allowed_collision_matrix)
        get_scene = Mock(return_value=Mock(scene=scene))
        apply = Mock(return_value=Mock(success=True))
        robot = Mock()
        robot.get_link_names.return_value = ['rh_palm', 'ra_forearm_link']
        with patch.object(sim_contacts.rospy, 'wait_for_service'), \
                patch.object(sim_contacts.rospy, 'ServiceProxy', side_effect=[get_scene, apply]):
            restore = sim_contacts.disable_replay_collision_checks(robot, protect_table=False)
            allowed = apply.call_args.args[0].scene.allowed_collision_matrix
            self.assertEqual(set(allowed.entry_names), {'rh_palm', 'floor', 'rect_object', 'held', 'ra_forearm_link'})
            self.assertTrue(all(all(row.enabled) for row in allowed.entry_values))
            self.assertTrue(all(allowed.default_entry_values))
            self.assertEqual(scene.allowed_collision_matrix, original)
            restore()
            self.assertEqual(apply.call_args.args[0].scene.allowed_collision_matrix, original)

    def test_real_replay_always_checks_table_and_retains_it_after_restore(self):
        from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
        for table in ('ground_plane_user', 'ground_plane_user__link'):
            scene = PlanningScene()
            scene.world.collision_objects = [CollisionObject(id=table), CollisionObject(id='rect_object')]
            scene.robot_state.attached_collision_objects = [AttachedCollisionObject(object=CollisionObject(id='held'))]
            # Even a stale all-allowed table rule must not survive the replay.
            scene.allowed_collision_matrix = contact_matrix(AllowedCollisionMatrix(), ['rh_palm'], table, True)
            get_scene = Mock(return_value=Mock(scene=scene))
            apply = Mock(return_value=Mock(success=True))
            robot = Mock()
            robot.get_link_names.return_value = ['rh_palm', 'ra_forearm_link']
            with patch.object(sim_contacts.rospy, 'wait_for_service'), \
                    patch.object(sim_contacts.rospy, 'ServiceProxy', side_effect=[get_scene, apply]):
                restore = sim_contacts.disable_replay_collision_checks(robot)
                active = apply.call_args.args[0].scene.allowed_collision_matrix
                obj, hand = [active.entry_names.index(n) for n in ('rect_object', 'rh_palm')]
                self.assertTrue(active.entry_values[obj].enabled[hand])
                restore()
                restored = apply.call_args.args[0].scene.allowed_collision_matrix
                for matrix in (active, restored):
                    index = matrix.entry_names.index(table)
                    for name in ('rh_palm', 'ra_forearm_link', 'held'):
                        other = matrix.entry_names.index(name)
                        self.assertFalse(matrix.entry_values[index].enabled[other])
                        self.assertFalse(matrix.entry_values[other].enabled[index])
                    self.assertFalse(matrix.default_entry_values[matrix.default_entry_names.index(table)])

    def test_real_replay_rejects_missing_table_before_changing_rules(self):
        from moveit_msgs.msg import PlanningScene
        get_scene = Mock(return_value=Mock(scene=PlanningScene()))
        apply = Mock()
        with patch.object(sim_contacts.rospy, 'wait_for_service'), \
                patch.object(sim_contacts.rospy, 'ServiceProxy', side_effect=[get_scene, apply]):
            with self.assertRaisesRegex(RuntimeError, 'requires the table'):
                sim_contacts.disable_replay_collision_checks(Mock())
        apply.assert_not_called()

    def test_stage_one_forces_checking_and_restores_after_success_or_failure(self):
        config = {'object': {'enabled': True, 'hand_collision': False}}
        for fail in (False, True):
            restore = Mock()
            with patch.object(sim_contacts, 'configure_hand_object_contacts', return_value=restore) as configure:
                try:
                    with sim_contacts.stage_one_contacts(Mock(), config):
                        self.assertTrue(configure.call_args.args[1]['object']['hand_collision'])
                        self.assertFalse(config['object']['hand_collision'])
                        restore.assert_not_called()
                        if fail:
                            raise RuntimeError('stage failed')
                except RuntimeError as exc:
                    self.assertTrue(fail)
                    self.assertEqual(str(exc), 'stage failed')
                restore.assert_called_once_with()

    def test_only_grasp_object_contacts_are_added(self):
        original = AllowedCollisionMatrix(
            entry_names=["finger", "floor"],
            entry_values=[AllowedCollisionEntry(enabled=[False, False]), AllowedCollisionEntry(enabled=[False, False])],
        )
        saved = copy.deepcopy(original)
        result = contact_matrix(original, ["finger", "palm"])
        self.assertEqual(original, saved)
        obj = result.entry_names.index("rect_object")
        floor = result.entry_names.index("floor")
        for name in ("finger", "palm"):
            link = result.entry_names.index(name)
            self.assertTrue(result.entry_values[obj].enabled[link])
            self.assertTrue(result.entry_values[link].enabled[obj])
            self.assertFalse(result.entry_values[link].enabled[floor])
        self.assertFalse(result.entry_values[obj].enabled[floor])

    def test_repeated_allowance_does_not_duplicate_names(self):
        first = contact_matrix(AllowedCollisionMatrix(), ["finger"])
        second = contact_matrix(first, ["finger"])
        self.assertEqual(first, second)

    def test_switch_only_changes_hand_object_pairs_and_preserves_arm_and_obstacles(self):
        original = AllowedCollisionMatrix(
            entry_names=['rh_palm', 'rh_ffdistal', 'ra_wrist_3_link', 'floor', 'rect_object'],
            entry_values=[AllowedCollisionEntry(enabled=[False] * 5) for _ in range(5)])
        for enabled in (True, False):
            config = {'object': {'hand_collision': enabled}}
            result = hand_object_matrix(original, original.entry_names[:3], config)
            obj = result.entry_names.index('rect_object')
            for hand in ('rh_palm', 'rh_ffdistal'):
                index = result.entry_names.index(hand)
                self.assertEqual(result.entry_values[obj].enabled[index], not enabled)
                self.assertEqual(result.entry_values[index].enabled[obj], not enabled)
                self.assertFalse(result.entry_values[index].enabled[result.entry_names.index('floor')])
            self.assertFalse(result.entry_values[obj].enabled[result.entry_names.index('ra_wrist_3_link')])
            self.assertTrue(all(not any(row.enabled) for row in original.entry_values))

    def test_reenabling_collisions_removes_previous_hand_allowance_including_scoped_ids(self):
        links = ['rh_palm', 'ra_wrist_3_link']
        allowed = hand_object_matrix(AllowedCollisionMatrix(), links, {'object': {'hand_collision': False}},
                                     ['rect_object::link::custom_collision', 'rect_object__custom_link'])
        checked = hand_object_matrix(allowed, links, {'object': {'hand_collision': True}})
        palm = checked.entry_names.index('rh_palm')
        for name in ('rect_object', 'rect_object::link', 'rect_object::link::collision',
                     'rect_object::link::custom_collision', 'rect_object__link',
                     'rect_object__link__collision', 'rect_object__custom_link'):
            obj = checked.entry_names.index(name)
            self.assertFalse(checked.entry_values[palm].enabled[obj])

    def test_disabled_switch_covers_gazebo_flattened_object_name(self):
        original = AllowedCollisionMatrix(entry_names=['ra_wrist_3_link'],
                                           entry_values=[AllowedCollisionEntry(enabled=[False])])
        matrix = hand_object_matrix(original, ['rh_ffdistal', 'ra_wrist_3_link'],
                                    {'object': {'hand_collision': False}})
        hand = matrix.entry_names.index('rh_ffdistal')
        obj = matrix.entry_names.index('rect_object__link')
        self.assertTrue(matrix.entry_values[obj].enabled[hand])
        self.assertFalse(matrix.entry_values[obj].enabled[matrix.entry_names.index('ra_wrist_3_link')])

    def test_configuration_restores_the_original_matrix_after_execution(self):
        original = AllowedCollisionMatrix()
        get_scene = Mock()
        get_scene.return_value.scene.allowed_collision_matrix = original
        get_scene.return_value.scene.world.collision_objects = []
        apply = Mock(return_value=Mock(success=True))
        robot = Mock()
        robot.get_link_names.return_value = ['rh_palm', 'ra_wrist_3_link']
        with patch.object(sim_contacts.rospy, 'wait_for_service'), \
                patch.object(sim_contacts.rospy, 'ServiceProxy', side_effect=[get_scene, apply]), \
                patch.object(sim_contacts.rospy, 'is_shutdown', return_value=False):
            restore = configure_hand_object_contacts(robot, {'object': {'enabled': True, 'hand_collision': False}})
            self.assertEqual(apply.call_count, 1)
            restore()
        self.assertEqual(apply.call_count, 2)
        self.assertEqual(apply.call_args.args[0].scene.allowed_collision_matrix, original)

    def test_no_hand_links_fails_instead_of_disabling_arm_contacts(self):
        with self.assertRaisesRegex(RuntimeError, 'no rh_ hand links'):
            hand_object_matrix(AllowedCollisionMatrix(), ['ra_wrist_3_link'], {'object': {}})

    def test_new_object_rows_preserve_non_hand_default_rules(self):
        original = AllowedCollisionMatrix(
            entry_names=['ra_wrist_3_link', 'rh_palm'],
            entry_values=[AllowedCollisionEntry(enabled=[True, False]),
                          AllowedCollisionEntry(enabled=[False, False])],
            default_entry_names=['ra_wrist_3_link'], default_entry_values=[True])
        matrix = hand_object_matrix(original, original.entry_names, {'object': {'hand_collision': True}})
        obj, arm, hand = (matrix.entry_names.index(name) for name in ('rect_object', 'ra_wrist_3_link', 'rh_palm'))
        self.assertTrue(matrix.entry_values[obj].enabled[arm])
        self.assertTrue(matrix.entry_values[arm].enabled[obj])
        self.assertFalse(matrix.entry_values[obj].enabled[hand])


if __name__ == "__main__":
    unittest.main()
