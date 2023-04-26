import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc

__author__ = 'Arash Adel'
__email__ = '<aaadel@umich.edu>'
__copyright__ = 'Copyright 2020, Arash Adel'

"""
**********************************************************************************
Brick Class
**********************************************************************************
"""


class Brick(object):

    def __init__(self, frame, dim, is_half_brick=False):
        """
        A Brick with the following main attributes:
        frame: rg.Plane
        dim:  (length, width, height)
        col, shape, material
        """
        self.frame = frame
        self.dim = dim
        if not is_half_brick:
            self.length = self.dim[0]
        else:
            self.length = self.dim[0]/2.0
        self.width = self.dim[1]
        self.height = self.dim[2]

        # Half Dimensions
        self.half_length = self.length / 2.0
        self.half_width = self.width / 2.0
        self.half_height = self.height / 2.0

        # Corner point of the brick
        self.points = []
        # Brick Brep
        self.shape = self.add_brick()

    def __str__(self):
        """
        Returns a message of the attributes of the brick.
        """
        message = "A brick with the following attributes: "
        cor_attr = "Coordinate = {}".format(tuple(self.frame.Origin))
        dim_attr = "dimensions = {}".format(self.dim)
        return message + cor_attr + " and " + dim_attr

    def add_brick(self):
        """
        Adds a Rhino box based on the center, orientation,
        and the dimennsions of the brick.
                pt_3------------------------pt_0
                    |          ^y          |
                    |          |           |
                    |          cp--->x     |
                    |                      |
                    |                      |
                pt_2------------------------pt_1
        """
        # Center Point
        cp = self.frame.Origin

        # Translation Vectors
        t_vec_x = rg.Vector3d.Multiply(self.frame.XAxis, self.half_length)
        t_vec_y = rg.Vector3d.Multiply(self.frame.YAxis, self.half_width)
        t_vec_z = rg.Vector3d.Multiply(self.frame.ZAxis, self.half_height)

        # Translation vectors on the plane of the brick
        t_vec_0 = rg.Vector3d.Add(t_vec_x, t_vec_y)
        t_vec_1 = rg.Vector3d.Add(t_vec_x, -t_vec_y)
        t_vec_2 = rg.Vector3d.Add(-t_vec_x, -t_vec_y)
        t_vec_3 = rg.Vector3d.Add(-t_vec_x, t_vec_y)

        # Corner points
        # Negative z
        pt_0 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_0, - t_vec_z))
        pt_1 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_1, - t_vec_z))
        pt_2 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_2, - t_vec_z))
        pt_3 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_3, - t_vec_z))

        # Positive z
        pt_4 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_0, t_vec_z))
        pt_5 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_1, t_vec_z))
        pt_6 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_2, t_vec_z))
        pt_7 = rg.Point3d.Add(cp, rg.Vector3d.Add(t_vec_3, t_vec_z))

        # List of corner points
        bott_points = [pt_0, pt_1, pt_2, pt_3]
        top_points = [pt_4, pt_5, pt_6, pt_7]
        self.points = bott_points + top_points

        brick_brep = rg.Brep.CreateFromBox(self.points)
        return brick_brep

    def draw_brick(self):
        """
        Adds the brep of the brick to the Rhino enviroment.
        """
        self.shape_id = sc.doc.Objects.AddBrep(self.shape)

    def set_col(self, new_col):
        """Setter for the new color.
        """
        self.col = new_col
        rs.ObjectColor(self.shape_id, self.col)

    def set_material(self, material=None):
        """
        Setter for the new material.
        """
        if material is None:
            self.material = self.col
        else:
            self.material = material
        mat_index = rs.AddMaterialToObject(self.shape_id)
        rs.MaterialColor(mat_index, self.material)

    def check_collision(self, other):
        """Checks the collision with another brick
        """
        bb_intersect = rg.Intersect.Intersection.BrepBrep(
            self.shape, other.shape, 0.001)
        return bb_intersect

    def rotate_brick(self, rot_val):
        """Rotates the brick around its zaxis by the rot_val

        Args:
            rot_val (float): value in radians
        """
        self.frame.Rotate(rot_val, self.frame.ZAxis, self.frame.Origin)
        self.shape = self.add_brick()

    def get_center_point(self):
        """Getter for the center point of the brick
        """
        return self.frame.Origin
