"""
clue_detection.py - finds the clue board in a camera image and straightens it.

This is NOT a ROS node. It's a plain Python module with the detection functions
(tested on saved photos with tools/contour_test.py), so the clue detector node
can import them and stay short.

Usage from a node in this package:
    from controller_pkg.clue_detection import detect_board, straighten_board

Main functions:
    detect_board(photo_bgr, min_board_area) -> 4 ordered corners of the board's
    grey middle [top-left, top-right, bottom-right, bottom-left], or None.

    straighten_board(photo_bgr, ordered_corners) -> a flat, straight-on picture
    of the grey middle, always BOARD_IMAGE_WIDTH x BOARD_IMAGE_HEIGHT pixels.
"""

import cv2
import numpy as np


# Blue border colour range (OpenCV HSV: hue 0-179, sat 0-255, val 0-255).
# Tuned with tools/hsv_test.py on 7 sign photos (2026-11-27):
#   border saturation >= 126, water saturation <= 60, so sat min = 90.
BLUE_LOWER = np.array([110, 90, 40])
BLUE_UPPER = np.array([130, 255, 255])

# A real board is a blue frame around a grey middle, so only part of it is blue.
# Measured: real boards 0.41-0.47, solid blue blocks/slabs (backs of signs) ~1.0.
MAX_BLUE_FRACTION = 0.7

# How much approxPolyDP may simplify an outline, as a fraction of its perimeter.
# 0.02 gave exactly 4 corners on all 7 photos.
CORNER_SIMPLIFY_FRACTION = 0.02

# A board whose outline comes within this many pixels of the image border is
# probably cut off (only partly in view), so its corners would be wrong. Skip it.
EDGE_MARGIN = 5

# Size of the straightened board picture, in pixels.
# The grey middle (clue banner) is 0.15 m x 0.10 m in Miti's clue board model,
# so the picture keeps the same 3 : 2 shape (600 / 400 = 0.15 / 0.10).
BOARD_IMAGE_WIDTH = 600
BOARD_IMAGE_HEIGHT = 400


def find_blue_contours(photo_bgr):
    """
    Input:  a colour photo (BGR).
    Output: (list of outer contours of the blue blobs, the blue mask).
    """
    photo_hsv = cv2.cvtColor(photo_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(photo_hsv, BLUE_LOWER, BLUE_UPPER)

    # RETR_EXTERNAL: only outermost outlines (skips the letters inside the board)
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    return contours, blue_mask


def measure_blue_fraction(contour, blue_mask):
    """
    Input:  one contour, and the blue mask it was found in.
    Output: fraction of the area inside the outline that is blue
            (~0.4 for a real board, ~1.0 for a solid blue block).
    """
    inside_outline = np.zeros_like(blue_mask)
    cv2.drawContours(inside_outline, [contour], -1, 255, cv2.FILLED)

    blue_inside_outline = cv2.bitwise_and(blue_mask, inside_outline)

    blue_pixel_count = cv2.countNonZero(blue_inside_outline)
    outline_area = cv2.contourArea(contour)
    if outline_area == 0:
        return 1.0                      # degenerate outline: treat as not a board
    return blue_pixel_count / outline_area


def touches_image_edge(contour, image_width, image_height):
    """
    Input:  one contour, and the image's width and height in pixels.
    Output: True if the contour comes within EDGE_MARGIN pixels of any image edge
            (so the board is probably cut off), otherwise False.
    """
    x, y, width, height = cv2.boundingRect(contour)

    touches_left = x <= EDGE_MARGIN
    touches_top = y <= EDGE_MARGIN
    touches_right = x + width >= image_width - EDGE_MARGIN
    touches_bottom = y + height >= image_height - EDGE_MARGIN

    return touches_left or touches_top or touches_right or touches_bottom


def find_grey_middle_corners(board_contour, blue_mask):
    """
    Finds the 4 corners of the board's grey middle (inside the blue frame).
    Uses the grey middle, not the outer outline, because the outer outline also
    includes the board's dark blue side edge when seen at an angle.

    Input:  the board's outer contour, and the blue mask.
    Output: 4 x 2 array of (x, y) corners (any order), or None if not 4 corners.
    """
    whole_board = np.zeros_like(blue_mask)
    cv2.drawContours(whole_board, [board_contour], -1, 255, cv2.FILLED)

    not_blue = cv2.bitwise_not(blue_mask)
    grey_middle = cv2.bitwise_and(whole_board, not_blue)

    middle_contours, hierarchy = cv2.findContours(grey_middle, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)
    if len(middle_contours) == 0:
        return None
    middle_contour = max(middle_contours, key=cv2.contourArea)

    perimeter = cv2.arcLength(middle_contour, True)
    simplified = cv2.approxPolyDP(middle_contour,
                                  CORNER_SIMPLIFY_FRACTION * perimeter, True)

    if len(simplified) != 4:
        return None
    return simplified.reshape(4, 2)


def order_corners(corners):
    """
    Input:  4 x 2 array of (x, y) points in any order.
    Output: 4 x 2 array in the order [top-left, top-right, bottom-right, bottom-left].
    (Image y grows downward: x+y smallest at top-left, y-x smallest at top-right.)
    """
    corner_sums = corners[:, 0] + corners[:, 1]
    corner_differences = corners[:, 1] - corners[:, 0]

    top_left = corners[np.argmin(corner_sums)]
    bottom_right = corners[np.argmax(corner_sums)]
    top_right = corners[np.argmin(corner_differences)]
    bottom_left = corners[np.argmax(corner_differences)]

    return np.array([top_left, top_right, bottom_right, bottom_left])


def straighten_board(photo_bgr, ordered_corners):
    """
    Turns the angled board in the photo into a flat, straight-on picture of its
    grey middle (inverse perspective transform), always the same size.

    Inputs:
        photo_bgr       - the camera image (BGR)
        ordered_corners - 4 x 2 array [TL, TR, BR, BL] of the grey middle's
                          corners in image pixels, as detect_board returns them
    Output: BGR image, BOARD_IMAGE_HEIGHT rows x BOARD_IMAGE_WIDTH columns.
    """
    # 1. Where the corners are now (in the photo). OpenCV needs float32 here.
    source_corners = ordered_corners.astype(np.float32)

    # 2. Where the corners should end up: the corners of the output picture,
    #    in the same order TL, TR, BR, BL. Last pixel index = size - 1.
    right_edge = BOARD_IMAGE_WIDTH - 1
    bottom_edge = BOARD_IMAGE_HEIGHT - 1
    destination_corners = np.array([
        [0, 0],                         # top-left
        [right_edge, 0],                # top-right
        [right_edge, bottom_edge],      # bottom-right
        [0, bottom_edge],               # bottom-left
    ], dtype=np.float32)

    # 3. Work out the 3 x 3 matrix that moves the source corners onto the
    #    destination corners (4 point pairs = exactly enough to solve it).
    transform_matrix = cv2.getPerspectiveTransform(source_corners,
                                                   destination_corners)

    # 4. Apply it to the whole photo. Only the area inside the 4 corners lands
    #    inside the output picture, so this also crops to the grey middle.
    #    Output size is given as (width, height).
    straight_board = cv2.warpPerspective(photo_bgr, transform_matrix,
                                         (BOARD_IMAGE_WIDTH, BOARD_IMAGE_HEIGHT))
    return straight_board


def detect_board(photo_bgr, min_board_area):
    """
    Finds the clue board in one camera image.

    Inputs:
        photo_bgr       - the camera image (BGR, as cv_bridge gives it)
        min_board_area  - smallest outline area (pixels) worth reading;
                          smaller boards are too far away to read

    Output: (ordered_corners, board_contour)
        ordered_corners - 4 x 2 array [TL, TR, BR, BL] of the grey middle, or None
        board_contour   - the board's outer outline (for drawing), or None
    If several blobs pass the checks, the biggest one is used (the closest sign).
    """
    contours, blue_mask = find_blue_contours(photo_bgr)
    image_height = photo_bgr.shape[0]      # shape is (rows, columns, colour channels)
    image_width = photo_bgr.shape[1]

    # 1. Collect every blob that passes the size, edge and grey-middle checks
    board_candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_board_area:
            continue
        if touches_image_edge(contour, image_width, image_height):
            continue
        blue_fraction = measure_blue_fraction(contour, blue_mask)
        if blue_fraction > MAX_BLUE_FRACTION:
            continue
        board_candidates.append(contour)

    if len(board_candidates) == 0:
        return None, None

    # 2. Keep the biggest one
    board_contour = max(board_candidates, key=cv2.contourArea)

    # 3. Find and order its 4 corners
    corners = find_grey_middle_corners(board_contour, blue_mask)
    if corners is None:
        return None, board_contour
    ordered_corners = order_corners(corners)
    return ordered_corners, board_contour