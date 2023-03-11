import os
cwd = os.getcwd()
file_path = os.path.dirname(__file__)
os.chdir(file_path)

from Faster_Delaunay import delaunay_boundary
from path_finding import path_finding
from splines import generate_increment_on_path
import numpy as np
from typing import List, Optional

os.chdir(cwd)


def control(cones: List[dict]):
    triangles, cones = delaunay_boundary(cones)
    midpoints = path_finding(triangles, cones)
    target_point = generate_increment_on_path(midpoints)
    theta = np.arctan(target_point[0] / target_point[1])

    return theta
