import Rhino.Geometry as rg
import scriptcontext as sc
import random as rand
import math
from brick import Brick
import helper_functions as hf

__author__ = 'Arash Adel'
__email__ = '<aaadel@umich.edu>'
__copyright__ = 'Copyright 2020, Arash Adel'

"""
**********************************************************************************
BrickWall Class
**********************************************************************************
"""


class BrickWall(object):

    def __init__(self, wall_brep, brick_dim, min_h_gap, v_gap):
        """
        A Brick with the following main attributes:
        wall_brep: underlying surface of the wall
        brick_dim: (length, width, height) of each brick
        min_h_gap: minimum horizontal gaps between bricks
        v_gap: vertical gaps between bricks
        num_bricks_in_row: number of bricks in a row
        bricks: a nested list: a list of rows, each row is a list of the row's bricks.
        """
        self.wall_brep = wall_brep
        self.brick_dim = brick_dim
        self.brick_length = self.brick_dim[0]
        self.half_brick_length = self.brick_length / 2.0
        self.min_h_gap = min_h_gap
        self.v_gap = v_gap

        # These are calculated later
        self.num_courses = None
        self.num_course_bricks = None
        self.course_crvs = None

        # Generating bricks
        self.bricks, self.courses = self.gen_brick_wall()

    def __str__(self):
        """
        Returns a message of the main attributes of the brick wall.
        """
        message = "A brick wall with the following attributes: "
        row_attr = "Number of courses = {}".format(self.num_courses)
        num_bricks_in_row_attr = "Maximum number of bricks in a course = {}".format(
            self.num_course_bricks)
        return message + row_attr + " and " + num_bricks_in_row_attr

    def _gen_courses_crvs(self):
        """Generates the underlying axis curve for each course of the wall.
        """
        low_pt, high_pt = hf.find_brep_low_high_face_points(self.wall_brep)
        start_pt = rg.Point3d(low_pt.X, low_pt.Y,
                              low_pt.Z + self.brick_dim[2]/2.0)
        end_pt = rg.Point3d(start_pt.X, start_pt.Y, high_pt.Z)
        interval = self.brick_dim[2]+self.v_gap
        courses_crvs = rg.Brep.CreateContourCurves(
            self.wall_brep, start_pt, end_pt, interval)
        self.num_courses = len(courses_crvs)
        # The next two lines are for adding course curves to Rhino
        # Set to True if you want to add  bricks points to Rhino
        add_to_Rhino = True
        if add_to_Rhino:
            for crv in courses_crvs:
                sc.doc.Objects.AddCurve(crv)
        return courses_crvs

    def _calc_num_course_bricks(self, courses_crvs):
        """Claculate the number of bricks in a row of the wall
        Note: With this stratery, the increase in the length of
        each course (compared to the minimum length) is
        abosrbed in the gaps between bricks.
        """
        # Find the even course with minimum length
        min_even_crv_length = float("inf")
        # Iterating over the even courses
        for course_index in range(0, len(courses_crvs), 2):
            course_crv = courses_crvs[course_index]
            crv_length = course_crv.GetLength()
            if crv_length < min_even_crv_length:
                min_even_crv_length = crv_length

        curr_iteration = 1
        gap_val = float("-inf")
        num_course_bricks = math.ceil(
            min_even_crv_length / self.brick_length)+1

        # Calculate the number of bricks in an even course
        while (gap_val < self.min_h_gap) and curr_iteration < 100:
            num_course_bricks -= 1
            num_gaps = num_course_bricks - 1
            gap_val = (min_even_crv_length - ((num_course_bricks) *
                                              self.brick_length))/num_gaps
            curr_iteration += 1
        return num_course_bricks

    def _find_shift_point(self, course_crv, crv_start, is_even):
        """Finds shift point for start or end of the wall
        """
        if crv_start:
            crv_pt = course_crv.PointAtStart
        else:
            crv_pt = course_crv.PointAtEnd

        if is_even:  # Even course
            # 1) Draw a circle with the radius of half a brick at each end
            shift_circle = rg.Circle(crv_pt, self.half_brick_length)
        else:  # Odd course
            # 1) Draw a circle with the radius of a brick + gap/2 at each end
            num_gaps = self.num_course_bricks - 1
            gap_val = (course_crv.GetLength() - ((self.num_course_bricks) *
                                                 self.brick_length))/num_gaps
            shift_circle = rg.Circle(
                crv_pt, self.brick_length + gap_val/2.0)

        # 2) Find intersection point
        intersect_event = rg.Intersect.Intersection.CurveCurve(
            course_crv, shift_circle.ToNurbsCurve(), 0.01, 0.01)
        intersect_pt = intersect_event[0].PointA
        return intersect_pt

    def _find_half_brick_cp(self, course_crv, crv_start):
        """Finds shift point for start or end of the wall
        """
        if crv_start:
            crv_pt = course_crv.PointAtStart
        else:
            crv_pt = course_crv.PointAtEnd
        # 1) Draw a circle with the half brick length
        shift_circle = rg.Circle(
            crv_pt, self.half_brick_length/2.0)
        # 2) Find intersection point
        intersect_event = rg.Intersect.Intersection.CurveCurve(
            course_crv, shift_circle.ToNurbsCurve(), 0.01, 0.01)
        half_brick_cp = intersect_event[0].PointA
        return half_brick_cp

    def _gen_course_bricks_cps(self, course_crv, is_even):
        """Generates the center points for the bricks of the course
        """
        # 1) Find shift points
        sp_intersect = self._find_shift_point(course_crv, True, is_even)
        ep_intersect = self._find_shift_point(course_crv, False, is_even)

        # 2) Trim the course curve
        trimmed_crv = course_crv.Trim(course_crv.ClosestPoint(
            sp_intersect)[1], course_crv.ClosestPoint(ep_intersect)[1])
        sc.doc.Objects.AddCurve(trimmed_crv)

        # 3) Divide the timmed curve to find the center points of bricks
        # Number of division is one less that the number of bricks in a course
        if is_even:
            num_bricks_current_course = self.num_course_bricks
        else:
            num_bricks_current_course = self.num_course_bricks - 1

        course_divisions = trimmed_crv.DivideByCount(
            num_bricks_current_course-1, True)
        course_bricks_cps = [
            trimmed_crv.PointAt(t) for t in course_divisions]

        if not is_even:
            # Add center points for half bricks to start and end
            st_half_brick_cp = self._find_half_brick_cp(course_crv, True)
            en_half_brick_cp = self._find_half_brick_cp(course_crv, False)
            # Inset the start half birck cp at the beginning of the list
            course_bricks_cps.insert(0, st_half_brick_cp)
            # Append the end half birck cp at the end of the list
            course_bricks_cps.append(en_half_brick_cp)
        # Set to True if you want to add  bricks points to Rhino
        add_to_Rhino = True
        if add_to_Rhino:
            for cpt in course_bricks_cps:
                sc.doc.Objects.AddPoint(cpt)
        return course_bricks_cps

    def gen_brick_wall(self):
        """Generates the brick wall
        """
        self.course_crvs = self._gen_courses_crvs()
        self.num_course_bricks = self._calc_num_course_bricks(self.course_crvs)
        bricks = []
        courses = []
        for course_index, course_crv in enumerate(self.course_crvs):
            # List of the bricks of the course
            course_bricks = []
            # Generate Bricks' Center Points
            if course_index % 2 == 0:
                is_even = True
            else:
                is_even = False
            bricks_cps = self._gen_course_bricks_cps(course_crv, is_even)
            # Generate Bricks
            bricks_cps_length = len(bricks_cps)
            for cp_index, brick_cp in enumerate(bricks_cps):
                # Brick's x axis is the tangent of the course curve at brick's cp
                rc, brick_frame = course_crv.FrameAt(
                    course_crv.ClosestPoint(brick_cp)[1])
                # Check that the direction of the Z axis of the frame is positive
                if rg.Vector3d.Multiply(brick_frame.ZAxis, rg.Vector3d(0, 0, 1.0)) < 0:
                    # Rotate the frame around its X axis 180 degrees
                    brick_frame.Rotate(3.141592653589793, brick_frame.XAxis)
                # Instantiate a brick with default dimensions
                if not is_even and (cp_index == 0 or cp_index == bricks_cps_length-1):
                    is_half_brick = True
                else:
                    is_half_brick = False
                brick = Brick(brick_frame, self.brick_dim, is_half_brick)
                # Append the brick to the course
                course_bricks.append(brick)
                # Append the brick to the list of bricks
                bricks.append(brick)
            # Append the list of course bricks to the bricks list
            courses.append(course_bricks)
        # return the nested list of bricks
        return bricks, courses

    def get_bricks(self):
        """Getter for bricks
        """
        return self.bricks

    def get_bricks_breps(self):
        """Getter for bricks' breps
        """
        bricks_breps = []
        for brick in self.bricks:
            bricks_breps.append(brick.shape)
        return bricks_breps

    def draw_bricks_with_material(self, render_dir, render_sequence=False):
        """Draws the brick wall and add color/material,
        and render each assembly step if render_sequene is set to True.
        """
        wall_bott_row_cp_z_val = self.bricks[0][0].frame.OriginZ
        wall_top_row_cp_z_val = self.bricks[-1][0].frame.OriginZ
        wall_row_z_range = wall_top_row_cp_z_val - wall_bott_row_cp_z_val
        for row_index, row in enumerate(self.bricks):
            # z coordinate of the origina of the brick's frame
            row_cp_z_val = row[0].frame.OriginZ
            g_b_percentage = (
                row_cp_z_val - wall_bott_row_cp_z_val)/wall_row_z_range
            g_b_val = g_b_percentage * 255
            row_col = [255, g_b_val, g_b_val]
            for brick_index, brick in enumerate(row):
                brick.draw_brick()
                brick.set_col(row_col)
                brick.set_material()

                if render_sequence:
                    # Renders the sequence
                    hf.render_step(render_dir, brick_index)

    def check_bricks_collision(self):
        """Check collisions between bricks of each row of the wall.
        """
        collision_col = [255, 255, 0]
        for row_index, row_bricks in enumerate(self.bricks):
            num_bricks = len(row_bricks)
            for brick_index, brick in enumerate(row_bricks):
                # Collision Check
                if brick_index < num_bricks-1:
                    adjacent_brick = row_bricks[brick_index+1]
                    if len(brick.check_collision(adjacent_brick)[1]) != 0:
                        brick.set_col(collision_col)
                        brick.set_material()
                        adjacent_brick.set_col(collision_col)
                        adjacent_brick.set_material()
