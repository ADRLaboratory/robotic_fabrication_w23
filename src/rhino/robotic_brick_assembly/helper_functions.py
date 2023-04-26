import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import os

__author__ = 'Arash Adel'
__email__ = '<aaadel@umich.edu>'
__copyright__ = 'Copyright 2020, Arash Adel'


"""
**********************************************************************************
# Helper Function
**********************************************************************************
"""


def check_layer(layer_name):
    """
    Checks if the layer exists.
    If the layer exists: deletes all the objects in the layer.
    """
    if rs.IsLayer(layer_name):
        rs.CurrentLayer(layer_name)
        # Delete all the elements in the layer
        current_pts = rs.ObjectsByLayer(layer_name, True)
        rs.DeleteObjects(current_pts)
    else:
        # Create the layer
        rs.AddLayer(name=layer_name)
        rs.CurrentLayer(layer_name)


def render_step(render_dir, sequence_num):
    """
    Renders the scene and saves the file.
    """
    rs.Command("_Render")
    # file name
    file_name = str(int(sequence_num)).zfill(5) + ".png"
    file_path = os.path.join(render_dir, file_name)
    # save the file
    rs.Command("_-SaveRenderWindowAs " + file_path)
    # Close the render window
    rs.Command("_-CloseRenderWindow")


def find_brep_low_high_face_points(ref_brep):
    """
    Returns a sorted list of brep_points.
    NOTE: brep should only have one face.
    """
    brep_edges = list(ref_brep.DuplicateEdgeCurves())
    brep_face_points = [brep_edge.PointAtStart for brep_edge in brep_edges]
    sorted_points = sorted(brep_face_points, key=lambda pt: pt.Z)
    return sorted_points[0], sorted_points[-1]
