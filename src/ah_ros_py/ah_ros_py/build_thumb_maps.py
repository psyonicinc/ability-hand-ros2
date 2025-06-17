import os
import pickle

import numpy as np
from ikpy.chain import Chain
from math import radians

from ah_ros_py.kd_tree_lookup import THUMB_INCREMENT
from ament_index_python.packages import get_package_share_directory


def main():
    print("Building thumb maps")
    right_thumb = Chain.from_urdf_file(
        os.path.join(
            get_package_share_directory("ah_urdf"),
            "urdf",
            "right_thumb_only.urdf",
        )
    )
    left_thumb = Chain.from_urdf_file(
        os.path.join(
            get_package_share_directory("ah_urdf"),
            "urdf",
            "left_thumb_only.urdf",
        )
    )
    increment = THUMB_INCREMENT
    positions_r = [[], []]
    positions_l = [[], []]

    for j1 in np.arange(0, 100.1, increment):
        for j2 in np.arange(0, 100.1, increment):
            pr = right_thumb.forward_kinematics(
                [0, -radians(j1), radians(j2), 0]
            )
            pl = left_thumb.forward_kinematics(
                [0, -radians(j1), radians(j2), 0]
            )
            positions_r[0].append([pr[0][3], pr[1][3], pr[2][3]])
            positions_l[0].append([pl[0][3], pl[1][3], pl[2][3]])
            positions_r[1].append([j1, j2])
            positions_l[1].append([j1, j2])

    with open("thumb_right.pkl", "wb") as f:
        pickle.dump(positions_r, f)

    with open("thumb_left.pkl", "wb") as f:
        pickle.dump(positions_l, f)


if __name__ == "__main__":
    main()
