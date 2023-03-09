from Faster_Delaunay import delaunay_boundary
from path_finding import path_finding
from splines import generate_increment_on_path
import numpy as np
from typing import List, Optional


def control(cones: List[dict], mode: str = "autocross", lap: Optional[int] = None,
            on_finishing_sprint: Optional[bool] = None, finish_line: Optional[np.ndarray] = None):
    distance_threshold_for_same_side = 1
    orange_cone_relevance_threshold = 10
    # At the finish line, how far can non-orange cones be away from orange ones and be on the same side?
    conversion_threshold = 5.5

    oranges = []
    n_removed = 0
    for i in range(len(cones)):
        if cones[i - n_removed]["Label"] == "Orange":
            oranges.append(cones.pop(i - n_removed))
    if len(oranges) > 0 and \
            np.linalg.norm(np.average(np.array([[orange["Xpos"], orange["Ypos"]] for orange in oranges]), axis=1)) < \
            orange_cone_relevance_threshold:
        for orange in oranges:
            for cone in cones:
                if np.linalg.norm([orange["Xpos"] - cone["Xpos"], orange["Ypos"] - cone["Ypos"]]) < conversion_threshold:
                    orange["Label"] = cone["Label"]
                    break
        oranges = []

    """if len(oranges) == 0 and on_finishing_sprint:
        ...  # we have lost track of the orange cones and should probably brake
    if len(oranges) == 1:
        ...
        # optional TODO: implement a way to determine on which side this cone is, i.e. does it follow the trajectory of
        # yellow or blue cones?
    if len(oranges) == 2:
        if np.linalg.norm(oranges[0] - oranges[1]) < \
                distance_threshold_for_same_side:
            # we only have cones on one side
            orange = np.average(oranges, axis=1)
        else:
            # the cones are on opposite sides
            ...
    elif len(oranges) == 3:
        if np.sqrt((oranges[0][0] - oranges[1][0]) ** 2 + (oranges[0][1] - oranges[1][1]) ** 2) < \
                distance_threshold_for_same_side:
            oranges = [np.average(oranges[:2], axis=1), oranges[2]]
        elif np.sqrt((oranges[0][0] - oranges[2][0]) ** 2 + (oranges[0][1] - oranges[2][1]) ** 2) < \
                distance_threshold_for_same_side:
            oranges = [np.average([oranges[0], oranges[2]], axis=1), oranges[1]]
        elif np.sqrt((oranges[2][0] - oranges[1][0]) ** 2 + (oranges[2][1] - oranges[1][1]) ** 2) < \
                distance_threshold_for_same_side:
            oranges = [np.average(oranges[1:], axis=1), oranges[0]]
    elif len(oranges) == 4:
        new_oranges = []
        if np.sqrt((oranges[0][0] - oranges[1][0]) ** 2 + (oranges[0][1] - oranges[1][1]) ** 2) < \
                distance_threshold_for_same_side:  # 0 - 1
            new_oranges.append(np.average([oranges[0], oranges[1]], axis=1))
        if np.sqrt((oranges[0][0] - oranges[2][0]) ** 2 + (oranges[0][1] - oranges[2][1]) ** 2) < \
                distance_threshold_for_same_side:  # 0 - 2
            new_oranges.append(np.average([oranges[0], oranges[2]], axis=1))
        if np.sqrt((oranges[2][0] - oranges[1][0]) ** 2 + (oranges[2][1] - oranges[1][1]) ** 2) < \
                distance_threshold_for_same_side:  # 2 - 1
            new_oranges.append(np.average([oranges[2], oranges[1]], axis=1))
        if np.sqrt((oranges[0][0] - oranges[3][0]) ** 2 + (oranges[0][1] - oranges[3][1]) ** 2) < \
                distance_threshold_for_same_side:  # 0 - 3
            new_oranges.append(np.average([oranges[0], oranges[3]], axis=1))
        if np.sqrt((oranges[3][0] - oranges[2][0]) ** 2 + (oranges[3][1] - oranges[2][1]) ** 2) < \
                distance_threshold_for_same_side:  # 3 - 2
            new_oranges.append(np.average([oranges[3], oranges[2]], axis=1))
        if np.sqrt((oranges[3][0] - oranges[2][0]) ** 2 + (oranges[3][1] - oranges[2][1]) ** 2) < \
                distance_threshold_for_same_side:  # 3 - 1
            new_oranges.append(np.average([oranges[3], oranges[1]], axis=1))"""

    triangles, cones = delaunay_boundary(cones)
    midpoints = path_finding(triangles, cones)
    target_point, max_theta = generate_increment_on_path(midpoints)
    theta = np.arctan(target_point[0] / target_point[1])

    return theta
