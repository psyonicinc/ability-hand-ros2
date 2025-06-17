import pickle
import os

from scipy.spatial import cKDTree

FINGER_INCREMENT = 0.1
THUMB_INCREMENT = 0.25
MOVE_THRESHOLD = 80


class KDLookup:
    """
    Class for doing target remapping on the ability hand fingers (not thumb)
    """

    def __init__(self, side: str = "Right", file_prefix=""):
        with open(
            os.path.join(
                file_prefix, "data", f"all_fingers_{side.lower()}.pkl"
            ),
            "rb",
        ) as f:
            self.finger_positions = pickle.load(f)

        with open(
            os.path.join(file_prefix, "data", f"thumb_{side.lower()}.pkl"),
            "rb",
        ) as f:
            self.thumb_positions = pickle.load(f)

        self.index_tree = cKDTree(self.finger_positions[0])
        self.middle_tree = cKDTree(self.finger_positions[1])
        self.ring_tree = cKDTree(self.finger_positions[2])
        self.pinky_tree = cKDTree(self.finger_positions[3])
        self.thumb_tree = cKDTree(self.thumb_positions[0])
        self.prev_thumb = [None, None]

    def lookup_finger(self, finger: int, position: list):
        """
        Returns the degrees required to get the finger anchor as close to that
        position as possible
        """
        if finger == 0:
            dist, idx = self.index_tree.query(position)
            return idx * FINGER_INCREMENT
        elif finger == 1:
            dist, idx = self.middle_tree.query(position)
            return idx * FINGER_INCREMENT
        elif finger == 2:
            dist, idx = self.ring_tree.query(position)
            return idx * FINGER_INCREMENT
        elif finger == 3:
            dist, idx = self.pinky_tree.query(position)
            return idx * FINGER_INCREMENT

    def lookup_thumb(self, position: list):
        """
        Returns the degrees required to get the thumb anchor as close to that
        position as possible
        """

        dist, idx = self.thumb_tree.query(position)
        j1 = idx // (100.0 / THUMB_INCREMENT + 1)
        j2 = idx % (100.0 / THUMB_INCREMENT + 1)

        # Prevent large jumps
        if self.prev_thumb[0] is None:
            self.prev_thumb[0] = j1
            self.prev_thumb[1] = j2

        if abs(j1 - self.prev_thumb[0]) + abs(j2 - self.prev_thumb[1]) > (
            MOVE_THRESHOLD * (1 / THUMB_INCREMENT)
        ):
            return float(self.prev_thumb[0] * THUMB_INCREMENT), float(
                self.prev_thumb[1] * THUMB_INCREMENT
            )
        else:
            self.prev_thumb[0] = j1
            self.prev_thumb[1] = j2
            return float(j1 * THUMB_INCREMENT), float(j2 * THUMB_INCREMENT)
