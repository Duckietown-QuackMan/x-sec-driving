import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
import os
import rospy
import json
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from shapely.geometry import (
    shape, Point,
    LineString, MultiLineString,
    Polygon, MultiPolygon,
)
from shapely.ops import unary_union
from typing import List, Tuple, Dict, Union, Optional


YML_HOMOGRAPHY = """
homography:
- 0.0
- 0.000
- 0.3
- -0.00172
- 0.0
- 0.5523
- 0.0000
- 0.011
- -2.0
"""

YML_HOMOGRAPHY_GIMPY = """
homography:
- 5.0031924330074045e-06
- 0.00021645860032971483
- 0.28760404952387253
- -0.0013372594732437718
- -3.0574937690114804e-07
- 0.43032129953977705
- -5.7114094999110746e-05
- 0.00939056491670396
- -1.0242595340532956
"""

H = np.array(yaml.safe_load(YML_HOMOGRAPHY_GIMPY)['homography']).reshape(3, 3)

# colors in this range can be considered as white
HSV_RANGES_WHITE_1 = [
    (0, 0, 150),
    (180, 100, 255),
]

# colors in this range can be considered as yellow
# HSV_RANGES_YELLOW = [
#     (25, 140, 100),
#     (45, 255, 255),
# ]

# colors in this range can be considered as red
# HSV_RANGES_RED_1 = [
#     (165, 140, 100),
#     (180, 255, 255),
# ]
# HSV_RANGES_RED_1 = [
#     (165, 70, 50),
#     (180, 255, 255),
# ]

HSV_RANGES_RED_1 = [
    (0, 140, 100),
    (15, 255, 255),
]
HSV_RANGES_RED_2 = [
    (165, 70, 50),
    (180, 255, 255),
]

# # --- Functions for plotting
# def plot_tile_bev(
#     tile: "Tile",
#     extra_info: str = "",
#     width_px: int = 640,
#     height_px: int = 480,
#     dpi: int = 100,
#     only_show: bool = False,
#     detected_white_segments: List[Tuple[float, float, float, float]] = [],
#     detected_yellow_segments: List[Tuple[float, float, float, float]] = [],
# ) -> Union[None, np.ndarray]:
#     """
#     Plots a top-down view (Bird's Eye View) of lane markings,
#     optionally along with detected line segments.

#     Parameters:
#         tile (Tile): A Tile object that contains lane markings.
#         extra_info (str): Extra information to be appended to the plot title (default is "").
#         width_px (int): The width of the plot in pixels (default is 640).
#         height_px (int): The height of the plot in pixels (default is 480).
#         dpi (int): The resolution of the plot in dots per inch (default is 100).
#         only_show (bool): If True, shows the plot using plt.show(). If False, returns the image as an ndarray (default is False).
#         detected_white_segments (List[Tuple[float, float, float, float]]): Detected white line segments to be plotted, where each segment is represented by (x1, y1, x2, y2).
#         detected_yellow_segments (List[Tuple[float, float, float, float]]): Detected yellow line segments to be plotted, where each segment is represented by (x1, y1, x2, y2).

#     Returns:
#         Union[None, np.ndarray]: If only_show is True, returns None; otherwise, returns the image as a NumPy array.
#     """
#     # Create the figure and axis with the given dimensions
#     fig, ax = plt.subplots(figsize=(width_px / dpi, height_px / dpi), dpi=dpi)

#     # Plot the tile lane markings (polygons) with corresponding colors
#     for poly_w_bgr in tile.markings():
#         poly, bgr = poly_w_bgr
#         b, g, r = bgr
#         rgb = (r, g, b)  # Convert BGR to RGB
#         # Extract x and y coordinates from the polygon
#         x, y = poly.exterior.xy
#         # Plot the polygon, swapping x and y to adjust orientation
#         ax.fill(np.array(y), np.array(x), alpha=1.0, color=np.array(rgb) / 255, edgecolor='black')

#     # Plot detected white segments in red
#     legend_added = False
#     for x1, y1, x2, y2 in detected_white_segments:
#         if not legend_added:
#             ax.plot([y1, y2], [x1, x2], color='red', linewidth=2, label="White Segments")
#             legend_added = True
#         else:
#             ax.plot([y1, y2], [x1, x2], color='red', linewidth=2)

#     # Plot detected yellow segments in blue
#     legend_added = False
#     for x1, y1, x2, y2 in detected_yellow_segments:
#         if not legend_added:
#             ax.plot([y1, y2], [x1, x2], color='blue', linewidth=2, label="Yellow Segments")
#             legend_added = True
#         else:
#             ax.plot([y1, y2], [x1, x2], color='blue', linewidth=2)

#     # Add legend if any segments are plotted
#     if len(detected_white_segments) or len(detected_yellow_segments):
#         ax.legend()

#     # Set the background color to black
#     ax.set_facecolor('black')
#     # Set aspect ratio to be equal for accurate representation
#     ax.set_aspect('equal')

#     # Set axis labels and title
#     ax.set_xlabel('Robot Baseline Y (left + / right -)')
#     ax.set_ylabel('Robot Baseline X (forward +)')
#     title = 'Top-Down View of Lane-markings'
#     if extra_info:
#         title += f" ({extra_info})"
#     ax.set_title(title)

#     # Set symmetric bounds for the plot
#     ax.set_xlim(-0.85, 0.85)  # Approximately covers the full tile area
#     ax.set_ylim(0.0, 0.85)

#     # Enable grid and invert the x-axis to point left
#     plt.grid(True)
#     plt.gca().invert_xaxis()

#     # Show or return the plot as per the only_show parameter
#     if only_show:
#         plt.show()
#     else:
#         # Convert the figure to a NumPy array and return as BGR format for OpenCV
#         np_img = matplotlib_to_numpy(fig)
#         plt.close(fig)
#         ret = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
#         return ret

def plot_lines(img: np.ndarray, lines: List[Tuple[int, int, int, int]], color_bgr: Tuple[int, int, int]) -> np.ndarray:
    """
    Draws lines on the input image with the specified color.

    Parameters:
        img (np.ndarray): The input image on which lines are to be drawn (in BGR format).
        lines (List[Tuple[int, int, int, int]]): A list of line segments represented by their end points (x1, y1, x2, y2).
        color_bgr (Tuple[int, int, int]): The color of the lines to be drawn in BGR format (e.g., (255, 0, 0) for blue).

    Returns:
        np.ndarray: The image with the lines drawn on it.
    """
    # Create a copy of the input image to avoid modifying the original
    ret = img.copy()

    # Iterate over each line segment in the list
    for line in lines:
        x1, y1, x2, y2 = line
        # Draw the line on the image using the specified color and thickness
        cv2.line(ret, (x1, y1), (x2, y2), color=color_bgr, thickness=4)

    # Return the modified image with the lines drawn
    return ret

def matplotlib_to_numpy(fig: plt.Figure) -> np.ndarray:
    """
    Converts a Matplotlib figure to a NumPy array.

    Parameters:
        fig (plt.Figure): The Matplotlib figure to be converted.

    Returns:
        np.ndarray: The resulting image represented as a NumPy array in RGB format.
    """
    # Attach a FigureCanvas to the figure for rendering it
    canvas = FigureCanvas(fig)
    canvas.draw()  # Draw the figure onto the canvas

    # Get the width and height of the figure to convert to a NumPy array
    width, height = canvas.get_width_height()

    # Convert the canvas drawing to a NumPy array in RGB format
    img = np.frombuffer(canvas.tostring_rgb(), dtype='uint8').reshape(height, width, 3)

    return img

# --- Functions for score evaluation
def buffer_borders(gt_lines: List[Union[LineString, MultiLineString]], buffer_size: float = 0.01) -> List[Polygon]:
    """
    Buffers the borders of input line geometries to create polygons with specified buffer size.

    Parameters:
        gt_lines (List[Union[LineString, MultiLineString]]): A list of Shapely LineString or MultiLineString geometries.
        buffer_size (float): The size of the buffer to apply to each line segment (default is 0.01).

    Returns:
        List[Polygon]: A list of Shapely Polygon objects representing the buffered borders.
    """
    buffered_polygons = []

    # Iterate over each line geometry in the input list
    for geom in gt_lines:
        # Extract individual line segments from the LineString or MultiLineString geometry
        segments = get_segments_from_ls_or_multils(geom)

        # Buffer each segment and add it to the list of buffered polygons
        for line in segments:
            buffered_polygons.append(line.buffer(buffer_size, cap_style=2))  # cap_style=2 for flat caps

    return buffered_polygons

def get_segments_from_linestring(linestring: LineString) -> List[LineString]:
    """
    Extracts individual 2-point line segments from a LineString as LineStrings.

    Parameters:
        linestring (LineString): The input LineString from which segments are extracted.

    Returns:
        List[LineString]: A list of LineString objects, each representing a 2-point segment.
    """
    line_segments = []
    coords = list(linestring.coords)

    # Iterate over consecutive pairs of coordinates to create segments
    for i in range(len(coords) - 1):
        point1 = coords[i]
        point2 = coords[i + 1]
        
        segment = LineString([point1, point2])
        line_segments.append(segment)

    return line_segments

def get_segments_from_ls_or_multils(data: Union[LineString, MultiLineString]) -> List[LineString]:
    """
    Extracts individual 2-point line segments from a LineString or MultiLineString.

    Parameters:
        data (Union[LineString, MultiLineString]): The input geometry, either LineString or MultiLineString.

    Returns:
        List[LineString]: A list of LineString objects, each representing a 2-point segment.
    """
    if data.geom_type == "LineString":
        return get_segments_from_linestring(data)
    elif data.geom_type == "MultiLineString":
        segments = []
        # Iterate over each LineString in the MultiLineString and extract segments
        for line in data.geoms:
            segments.extend(get_segments_from_linestring(line))
        return segments

def plot_shapely_geometries(
    geometries: List[Union[Point, LineString, Polygon, MultiPolygon]],
    colors: Optional[List[str]] = None,
    alphas: Optional[List[float]] = None,
    extra_info: str = "",
    only_show: bool = False,
    ax: Optional[plt.Axes] = None,
) -> Optional[np.ndarray]:
    """
    Plots a list of Shapely geometries (Point, LineString, Polygon, MultiPolygon) using Matplotlib.

    Parameters:
        geometries (List[Union[Point, LineString, Polygon, MultiPolygon]]): A list of Shapely geometry objects to be plotted.
        colors (Optional[List[str]]): List of colors for each geometry. If None, all geometries will be green (default is None).
        alphas (Optional[List[float]]): List of alpha (transparency) values for each geometry. If None, default value is 0.5 for all (default is None).
        extra_info (str): Extra information to be appended to the plot title (default is "").
        only_show (bool): If True, shows the plot using plt.show(). If False, returns the image as an ndarray (default is False).
        ax (Optional[plt.Axes]): An existing Matplotlib axis to draw the geometries on. If None, creates a new one (default is None).

    Returns:
        Optional[np.ndarray]: If only_show is True, returns None; otherwise, returns the image as a NumPy array in BGR format.
    """
    if ax is None:
        fig, ax = plt.subplots()

    # Validate or assign default colors and alphas
    if colors is not None:
        assert len(geometries) == len(colors)
    else:
        colors = ['green'] * len(geometries)  # Default color for all geometries

    if alphas is not None:
        assert len(geometries) == len(alphas)
    else:
        alphas = [0.5] * len(geometries)  # Default transparency

    face_color = "black"

    # Iterate over each geometry and plot based on its type
    for geom, color, alpha in zip(geometries, colors, alphas):
        if geom.geom_type == 'Point':
            ax.plot(geom.y, geom.x, 'o', color=color, markersize=5, label='Point', alpha=alpha)
        elif geom.geom_type == 'LineString':
            x, y = geom.xy
            ax.plot(y, x, color=color, linewidth=2, linestyle='-', label='LineString', alpha=alpha)
        elif geom.geom_type == 'Polygon':
            x, y = geom.exterior.xy
            ax.fill(y, x, alpha=alpha, color=color, label='Polygon')
            # Plot interiors (holes) of the polygon in black
            for interior in geom.interiors:
                x, y = interior.xy
                ax.fill(y, x, alpha=alpha, color=face_color, label='Polygon Interior')
        elif geom.geom_type == 'MultiPolygon':
            for poly in geom.geoms:
                x, y = poly.exterior.xy
                ax.fill(y, x, alpha=alpha, color=color, label='MultiPolygon Part')
                # Plot interiors (holes) of the polygon in black
                for interior in poly.interiors:
                    x, y = interior.xy
                    ax.fill(y, x, alpha=alpha, color=face_color, label='Polygon Interior')

    # Set the background color to black
    ax.set_facecolor(face_color)
    # Set aspect ratio to be equal for accurate representation
    ax.set_aspect('equal')

    # Set labels and title
    ax.set_xlabel('Robot Baseline Y (left + / right -)')
    ax.set_ylabel('Robot Baseline X (forward +)')
    title = 'Line Detection IoU'
    if extra_info:
        title += f" ({extra_info})"
    ax.set_title(title)

    # Set symmetric bounds for the plot
    ax.set_xlim(-0.85, 0.85)  # Approximately covers the full tile area
    ax.set_ylim(0.0, 0.85)

    # Enable grid and invert the x-axis to point left
    plt.grid(True)
    plt.gca().invert_xaxis()

    # Show or return the plot as per the only_show parameter
    if only_show:
        plt.show()
        return None
    else:
        np_img = matplotlib_to_numpy(fig)
        plt.close(fig)
        ret = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
        return ret

def evaluate(
    tile: "XSecTile",
    detected_ground_segment: List[Tuple[float, float, float, float]],
    eval: bool,
    buffer_size: float = 0.005,
    score_radius: float = 0.25,
    passing_threshold: float = 0.015,
) -> Tuple[float, np.ndarray]:
    """
    Evaluates the detected line segments against ground truth lane markings, calculating an Intersection over Union (IoU) score.

    Parameters:
        tile (Tile): A Tile object containing information about ground truth polygons.
        ground_segment (List[Tuple[float, float, float, float]]): Detected white line segments represented by (x1, y1, x2, y2).
        yellow_ground_segments (List[Tuple[float, float, float, float]]): Detected yellow line segments represented by (x1, y1, x2, y2).
        buffer_size (float): Padding applied to line segments before calculating IoU (default is 0.005).
        score_radii (List[float]): List of radii from the origin at which scores are calculated, with closer detections given higher weight.
        passing_threshold (float): Score threshold to determine whether detection passes (default is 0.55).

    Returns:
        Tuple[float, np.ndarray]: The evaluation score and an image visualization of the results.
    """
    # Extract data identifier and ground truth polygons
    gt_polygons = tile.x_sec_polygon

    # Convert segments to buffered polygons, and find their union
    detected_seg_polygons = [LineString([(x1, y1), (x2, y2)]).buffer(buffer_size, cap_style=2) for x1, y1, x2, y2 in detected_ground_segment]
    detection_union = unary_union(detected_seg_polygons)

    # Buffer ground truth polygons to create ground truth unions
    gt_all = [poly.boundary for poly in gt_polygons]
    bf = buffer_borders(gt_all, buffer_size=buffer_size)
    gt_union = unary_union(bf)

    # Ensure geometries are valid before computing IoU scores
    assert detection_union.is_valid, "The Union geometry of [Detected White Lines] is not valid!"
    assert gt_union.is_valid, "The Union geometry of White Ground Truth is not valid!"

    # Calculate IoU scores for each radius and visualize
    circle = Point(0,0).buffer(score_radius)

    # Intersect the unions with the scoring circle
    gt = gt_union.intersection(circle)
    det = detection_union.intersection(circle)
    
    # Calculate intersection and union areas for IoU
    inter = gt.intersection(det).area
    union = gt.union(det).area
    score = inter / float(union)

    # Visualize the results for the current radius
    if eval:
        bgr = plot_shapely_geometries(
            [gt, det, circle],
            colors=["blue", "yellow", "red"],
            alphas=[0.8, 0.5, 0.8],
            extra_info=f"R={score_radius}, IoU={score:.4f})"
        )
    else:
        bgr = []
   
    passed = (score >= passing_threshold)
    
    # print("GT union [cm2]: ", round(gt.area * 1e2, 3), " Det union [cm2]: ", round(det.area * 1e2, 3))
    # print("Intersec [cm2]: ", round(inter * 1e2, 3), " union [cm2]: ", round(union * 1e2, 3))
    if passed:
        rospy.loginfo(f"Score: {round(score,3)}")

    return score, bgr, passed



# -- Image Process Function

def crop_image(img: np.ndarray) -> np.ndarray:
    """
    Parameters:
        img (np.ndarray): The input image to be cropped.

    Returns:
        np.ndarray: The cropped image
    """

    top=200
    bottom=0
    left=0 
    right=0

    return img[top:img.shape[0] - bottom, left:img.shape[1] - right]

def smoothen_image(
    img: np.ndarray,
) -> np.ndarray:
    """
    Parameters:
        img (np.ndarray): The input image to be smoothened (in BGR format).

    Returns:
        np.ndarray: The smoothened image (in BGR format).
    """
    ksize = 5
    threshold = 100
    # Apply Gaussian blur to the image
    smooth_img = cv2.GaussianBlur(img, (ksize, ksize), 0)
    # Convert the image to grayscale
    gray_img = cv2.cvtColor(smooth_img, cv2.COLOR_BGR2GRAY)
    # Apply binary thresholding to make it black and white
    _, bw_img = cv2.threshold(gray_img, threshold, 255, cv2.THRESH_BINARY)

    return bw_img

def find_edges(img: np.ndarray) -> np.ndarray:
    """
    Parameters:
        img (np.ndarray): The input image (in grayscale or BGR format).

    Returns:
        np.ndarray: An image with edges highlighted (single-channel binary image).
    """
    low_threshold = 50
    high_threshold = 150
    # Use the Canny edge detector
    edges = cv2.Canny(img, low_threshold, high_threshold)
    return edges

def dilate_edge_mask(edge_mask: np.ndarray) -> np.ndarray:
    """
    Parameters:
        edge_mask (np.ndarray): The input edge mask (binary image, from edge detection).

    Returns:
        np.ndarray: The dilated edge mask.
    """

    kernel_size = 15
    iterations = 1
    # Create a square kernel of specified size
    kernel = np.ones((kernel_size, kernel_size), np.uint8)

    # Apply dilation
    dilated_mask = cv2.dilate(edge_mask, kernel, iterations=iterations)

    return dilated_mask

def color_dilated_edges(
    img: np.ndarray,
    dilated_edge_mask: np.ndarray
) -> np.ndarray:
    """
    Parameters:
        img (np.ndarray): The input image (in BGR format).
        dilated_edge_mask (np.ndarray): The binary mask representing dilated edges (single-channel).

    Returns:
        np.ndarray: The image with colored edges, masked by the dilated edge mask.
    """

    # Convert 2D mask to 3-channel
    dilated_edge_mask_3d = np.dstack([dilated_edge_mask] * 3)

    # Only show the original color where the mask is active
    overlayed_img = np.where(dilated_edge_mask_3d > 0, img, 0)

    return overlayed_img

def edges_in_color_range(
    colored_edges: np.ndarray,
    low1: Tuple[int, int, int],
    high1: Tuple[int, int, int],
    low2: Tuple[int, int, int],
    high2: Tuple[int, int, int],
) -> np.ndarray:
    """
    Parameters:
        colored_edges (np.ndarray): The input image (in BGR format).
        low (Tuple[int, int, int]): The lower bound of the color range in HSV (e.g., (H, S, V)).
        high (Tuple[int, int, int]): The upper bound of the color range in HSV (e.g., (H, S, V)).

    Returns:
        np.ndarray: A binary mask where the pixels within the specified color range are 1, and others are 0.
    """

    # Convert the BGR image to HSV color space
    hsv_image = cv2.cvtColor(colored_edges, cv2.COLOR_BGR2HSV)
    
    # Generate a binary mask where pixels within the color range are 255, and others are 0
    mask1 = cv2.inRange(hsv_image, low1, high1)
    mask2 = cv2.inRange(hsv_image, low2, high2)

    # Combine the two masks
    mask = cv2.bitwise_or(mask1, mask2)
    return mask

def detect_lines(edges: np.ndarray) -> List[np.ndarray]:
    """
    Parameters:
        edges (np.ndarray): The edge-detected image (single-channel binary image).

    Returns:
        List[np.ndarray]: A list of line segments, where each segment is represented by an array [x1, y1, x2, y2].
                          Returns an empty list if no lines are detected.
    """

    # Use the Hough Line Transform to detect lines
    lines = cv2.HoughLinesP(
        edges,
        rho=10,                       # Distance resolution of the accumulator in pixels
        theta= 2 * np.pi / 180,      # Angle resolution of the accumulator in radians
        threshold=10,                  # Accumulator threshold for line detection
        minLineLength=10,             # Minimum length of line. Shorter lines are rejected
        maxLineGap=50                 # Maximum allowed gap between line segments to treat them as single line
    )

    # --- keep following ---
    # If lines are detected, reshape the array for easier handling
    if lines is not None:
        lines = lines.reshape((-1, 4))  # Reshape to get a list of 4-element arrays [x1, y1, x2, y2]
    else:
        lines = []

    return lines

def project_point_image_to_ground(H: np.ndarray, pixel_coord: Tuple[int, int]) -> Tuple[float, float]:
    """
    Parameters:
        H (np.ndarray): The homography matrix (3x3) used to transform points from the image to ground.
        pixel_coord (Tuple[int, int]): The pixel coordinates in the image plane (x, y).

    Returns:
        Tuple[float, float]: The projected ground plane coordinates (x, y).
    """
    # Convert pixel coordinates to homogeneous form (x, y, 1)
    pixel_homogeneous = np.array([*pixel_coord, 1])

    # Apply the homography matrix
    ground_coord_homogeneous = H @ pixel_homogeneous.T

    # Normalize to convert from homogeneous to Cartesian coordinates
    x_ground = ground_coord_homogeneous[0] / ground_coord_homogeneous[2]
    y_ground = ground_coord_homogeneous[1] / ground_coord_homogeneous[2] #correction due to side cropping

    return x_ground, y_ground

def project_segments_image_to_ground(
    H: np.ndarray,
    line_segments: List[Tuple[int, int, int, int]],
    offset_x: int = 0,
    offset_y: int = 0,
) -> List[Tuple[float, float, float, float]]:
    """
    Projects line segments from the image plane to the ground plane using a homography matrix, with optional offsets.

    Parameters:
        H (np.ndarray): The homography matrix (3x3) used to transform points from the image plane to the ground plane.
        line_segments (List[Tuple[int, int, int, int]]): List of line segments represented by their end points (x1, y1, x2, y2).
        offset_x (int): The offset to be added to the x-coordinates of the line segments before projection (default is 0).
        offset_y (int): The offset to be added to the y-coordinates of the line segments before projection (default is 0).

    Returns:
        List[Tuple[float, float, float, float]]: The projected line segments in the ground plane,
        each represented by their start and end points (x1, y1, x2, y2).
    """
    ground_segments = []

    # Iterate over each line segment to apply the homography
    for seg in line_segments:
        # Adjust coordinates by adding the provided offsets before projection
        p0 = project_point_image_to_ground(H, (seg[0] + offset_x, seg[1] + offset_y))
        p1 = project_point_image_to_ground(H, (seg[2] + offset_x, seg[3] + offset_y))

        # Create a tuple representing the projected segment in the ground plane
        gnd_seg = (*p0, *p1)

        # Append the projected segment to the result list
        ground_segments.append(gnd_seg)

    return ground_segments

def line_detection(image: np.ndarray) -> Tuple[List[Tuple[float, float, float, float]], List[Tuple[float, float, float, float]]]:
    """
    Performs full line detection on an input image, including preprocessing, edge detection, line detection and projection to ground plane.

    Parameters:
        image (np.ndarray): The input image on which line detection is to be performed.

    Returns:
        Tuple[List[Tuple[float, float, float, float]], List[Tuple[float, float, float, float]]]: 
        Returns two lists of tuples representing the ground segments for white and yellow lines, respectively.
        Each tuple represents a line segment as (x1, y1, x2, y2).
    """
    
    # Step 1: Crop the image
    cropped_image = crop_image(image)

    # Step 2: Smooth the image
    smooth_image = smoothen_image(cropped_image)

    # Step 3: Find edges
    edges = find_edges(smooth_image)

    # Step 4: Dilate the edge mask
    dilated_edges = dilate_edge_mask(edges)
    
    # Step 5: Color Dilated edge mask
    colored_edges = color_dilated_edges(cropped_image, dilated_edges)

    # Step 6: Extract colored edges for red
    red_edges = edges_in_color_range(colored_edges, HSV_RANGES_RED_1[0], HSV_RANGES_RED_1[1], HSV_RANGES_RED_2[0], HSV_RANGES_RED_2[1])

    # Step 7: Detect lines from the dilated edge mask
    red_lines = detect_lines(red_edges)

    # Step 8: Project the detected line segments to the ground plane
    projected_red_lines = project_segments_image_to_ground(H, red_lines, offset_y=180)

    return red_edges, red_lines, projected_red_lines, cropped_image


def redline_detection(
    img: np.ndarray,
    threshold: float = 400000,
    ) -> np.array:
    """
    Parameters:
        img (np.ndarray): The input image to be cropped.

    Returns:
        float: score of redline detection in image
    """
    
    top = 350
    bottom = 0
    left = 50 
    right = 0
    
    #crop the image
    cropped_image = img[top:(img.shape[0] - bottom), left:(img.shape[1]- right)]
    # Convert the BGR image to HSV color space
    hsv_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    # Create red masks (two ranges for red hue wrapping around 0 degrees)
    red_mask1 = cv2.inRange(hsv_image, HSV_RANGES_RED_1[0], HSV_RANGES_RED_1[1])
    red_mask2 = cv2.inRange(hsv_image, HSV_RANGES_RED_2[0], HSV_RANGES_RED_2[1])
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    if np.sum(red_mask) > threshold:
        rospy.loginfo(f"RED detection {np.sum(red_mask)}")
        return True, hsv_image
    
    return False , hsv_image


# --- Data structure to hold input data and ground truth
class Tile:
    def __init__(
        self,
        dir_path: str,  # path where the data directory is
        fpv_fname: str = "fpv.png",
        polygons_fname: str = "polygons_with_bgr.json",
        config_fname: str = "config.json"
    ):
        """
        Initializes the Tile object by loading the FPV image, ground truth polygons, and configuration data.

        Parameters:
            dir_path (str): Path to the directory containing data files.
            fpv_fname (str): Filename for the first-person view (FPV) image (default is "fpv.png").
            polygons_fname (str): Filename for the JSON file containing polygons with BGR colors (default is "polygons_with_bgr.json").
            config_fname (str): Filename for the JSON file containing configuration data (default is "config.json").
        """
        # Load the FPV image
        p_fpv = os.path.join(dir_path, fpv_fname)
        self._fpv_image = cv2.imread(p_fpv)

        # Load ground truth polygons and associated colors
        p_polygons = os.path.join(dir_path, polygons_fname)
        with open(p_polygons, 'r') as f:
            loaded_data = json.load(f)
        self._lst_poly_w_bgr = []
        for item in loaded_data:
            polygon = shape(item["polygon"])
            color = tuple(item["color"])
            self._lst_poly_w_bgr.append((polygon, color))

        # Load configuration data
        p_config = os.path.join(dir_path, config_fname)
        with open(p_config, 'r') as f:
            self._config = json.load(f)

    def fpv(self) -> 'np.ndarray':
        """
        Returns the FPV image.

        Returns:
            np.ndarray: The loaded FPV image.
        """
        return self._fpv_image

    def data_id(self) -> str:
        """
        Returns the ID from the configuration data.

        Returns:
            str: The data identifier.
        """
        return self._config.get("ID", "unset")

    def disp_config(self) -> None:
        """
        Displays the configuration data in JSON format.
        """
        print(json.dumps(self._config, indent=2))

    def markings(self) -> List[Tuple[Polygon, Tuple[int, int, int]]]:
        """
        Returns all the markings in the tile, each with an associated color.

        Returns:
            List[Tuple[Polygon, Tuple[int, int, int]]]: A list of polygons with their associated BGR color.
        """
        return self._lst_poly_w_bgr

    def white_polygons(self) -> List[Polygon]:
        """
        Returns a list of ground polygons that are white.

        Raises:
            Exception: If the FPV image has not been generated.
        """
        return [poly for poly, color in self._lst_poly_w_bgr if color == (255, 255, 255)]

    def yellow_polygons(self) -> List[Polygon]:
        """
        Returns a list of ground polygons that are yellow.

        Returns:
            List[Polygon]: A list of yellow polygons.
        """
        return [poly for poly, color in self._lst_poly_w_bgr if color == (0, 255, 255)]

class XSecTile():
    def __init__(self):
        self.x_sec_polygon : List[Polygon] = [Polygon(((0.175, -0.10), (0.175, 0.10), (0.225, 0.10), (0.225, -0.10), (0.175, -0.10)))]
        self.x_sec_color = (255, 0, 0)
    def markings(self) -> List[Tuple[Polygon, Tuple[int, int, int]]]:
        return (self.x_sec_polygon, self.x_sec_color)
