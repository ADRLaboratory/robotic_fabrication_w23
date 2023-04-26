
import sys
import math
from robotic_brick_assembly.brick_wall import BrickWall
import Rhino.Geometry as rg

my_path = r'path\to\mymodule.py'
if my_path not in sys.path:
    sys.path.append(my_path)


__author__ = 'Arash Adel'
__email__ = '<aaadel@umich.edu>'
__copyright__ = 'Copyright 2020, Arash Adel'


"""
**********************************************************************************
Brick Wall
**********************************************************************************
"""


def main():
    """
    Main body
    """
    global wall_brep, attractor_point, max_rotation
    global BrickWall_01, bricks, bricks_breps
    # brick_dim = (length, width, height)
    brick_dim = (0.050, 0.025, 0.01905)

    min_h_gap = 0.002
    v_gap = 0.0

    if "wall_brep" not in globals():
        wall_brep = None

    if "max_rotation" not in globals():
        max_rotation = 0.0

    # Instantiate a BrickWall
    BrickWall_01 = BrickWall(wall_brep, brick_dim, min_h_gap, v_gap)
    bricks = BrickWall_01.get_bricks()
    bricks_breps = BrickWall_01.get_bricks_breps()

    # Rotate bricks based on their distane to an attractor point
    max_rotation_rad = math.radians(max_rotation)

    bricks_cps = [brick.get_center_point() for brick in bricks]
    cps_dists = [brick_cp.DistanceTo(attractor_point)
                 for brick_cp in bricks_cps]
    max_dist = max(cps_dists)
    for brick in bricks:
        brick_cp = brick.get_center_point()
        dist = brick_cp.DistanceTo(attractor_point)
        ratio = dist / max_dist
        brick.rotate_brick(ratio*max_rotation_rad)
        # print(dist)
    bricks_breps = BrickWall_01.get_bricks_breps()


# Run the code
if __name__ == '__main__':
    main()
