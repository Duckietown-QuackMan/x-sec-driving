import numpy as np
from typing import List, Tuple
from shapely.geometry import (
    shape, Point,
    LineString, MultiLineString,
    Polygon, MultiPolygon,
)

# import the IMPLEMENTED FUNCTIONS
from XsecDetector import (
    plot_tile_bev,
    plot_shapely_geometries,
    crop_image,
    smoothen_image,
    find_edges,
    dilate_edge_mask,
    color_dilated_edges,
    edges_in_color_range,
    detect_lines,
    project_segments_image_to_ground,
    Tile,
    H,
    HSV_RANGES_RED
)


class XSecTile():
    def __init__(self):
        self.x_sec_polygon = Polygon(((0., 0.), (0., -0.21), (0.1, -0.21), (0.1, 0.), (0., 0.)))
        self.x_sec_color = (255, 0, 0)
xsec = XSecTile()
plot_shapely_geometries(xsec.x_sec_polygon, ['red'])