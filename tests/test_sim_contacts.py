import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "app"))
from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
from sim_contacts import contact_matrix


class ContactTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
