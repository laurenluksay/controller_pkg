"""
contour_test.py - step 2a of clue board detection: find the outlines of the blue blobs.

Not a ROS node. For each saved sign photo it:
  1. makes the blue mask (same HSV range as hsv_test.py)
  2. finds the outer outline (contour) of every white blob in the mask
  3. draws each outline on the photo with a number, and prints its area

Run it:
    python3 contour_test.py ~/sign_frames

Keys (click on the image window first):
    n = next photo
    p = previous photo
    q = quit
"""

import glob
import os
import sys

import cv2
import numpy as np


# Blue border colour range, tuned with hsv_test.py (2026-11-27)
BLUE_LOWER = np.array([110, 90, 40])
BLUE_UPPER = np.array([130, 255, 255])

# Blobs smaller than this many pixels are not drawn (letters, specks)
MIN_AREA_TO_SHOW = 300

# A real board is a blue frame around a grey middle, so only part of it is blue.
# Measured 2026-11-27: real boards 0.41-0.47, solid blue blocks/slabs 1.0 or more.
MAX_BLUE_FRACTION = 0.7

# How much approxPolyDP may simplify an outline, as a fraction of its perimeter.
# Bigger = fewer corners kept. 0.02 gave exactly 4 corners on all 7 photos.
CORNER_SIMPLIFY_FRACTION = 0.02

WINDOW_NAME = 'Contours: every blue blob, with its number'


def find_blue_contours(photo_bgr):
    """
    Input:  a colour photo (BGR).
    Output: (list of contours, the blue mask).
            Each contour is a list of (x, y) points around one blob's outer edge.
    """
    photo_hsv = cv2.cvtColor(photo_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(photo_hsv, BLUE_LOWER, BLUE_UPPER)

    # RETR_EXTERNAL = only the outermost outline of each blob.
    #   The letters sit inside the board's hole, so their outlines are skipped.
    # CHAIN_APPROX_SIMPLE = store only the corner points of straight edges, not every pixel.
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    return contours, blue_mask


def measure_blue_fraction(contour, blue_mask):
    """
    Input:  one contour, and the blue mask it was found in.
    Output: the fraction of the area inside the outline that is blue (0 to about 1).
            About 0.4 for a real board (grey middle), about 1.0 for a solid blue block.
    """
    # 1. Make an all-black image the same size as the mask, and fill this contour in white
    inside_outline = np.zeros_like(blue_mask)
    cv2.drawContours(inside_outline, [contour], -1, 255, cv2.FILLED)

    # 2. Keep only pixels that are BOTH inside the outline AND blue
    blue_inside_outline = cv2.bitwise_and(blue_mask, inside_outline)

    # 3. Count them and divide by the outline's area
    blue_pixel_count = cv2.countNonZero(blue_inside_outline)
    outline_area = cv2.contourArea(contour)
    blue_fraction = blue_pixel_count / outline_area
    return blue_fraction


def find_grey_middle_corners(board_contour, blue_mask):
    """
    Finds the 4 corners of a board's grey middle (the area inside the blue frame).
    We use the grey middle, not the outer blue outline, because the outer outline
    also includes the board's dark blue side edge when it's seen at an angle (sign 7).

    Input:  the board's outer contour, and the blue mask.
    Output: a 4 x 2 numpy array of (x, y) corner points, or None if it isn't 4 corners.
    """
    # 1. Fill in the whole board (frame + middle) in white
    whole_board = np.zeros_like(blue_mask)
    cv2.drawContours(whole_board, [board_contour], -1, 255, cv2.FILLED)

    # 2. Remove the blue pixels: what's left is the grey middle (plus the letters' holes)
    not_blue = cv2.bitwise_not(blue_mask)
    grey_middle = cv2.bitwise_and(whole_board, not_blue)

    # 3. Outline the grey middle. Keep the biggest outline, in case of small leftover specks.
    middle_contours, hierarchy = cv2.findContours(grey_middle, cv2.RETR_EXTERNAL,
                                                  cv2.CHAIN_APPROX_SIMPLE)
    if len(middle_contours) == 0:
        return None
    middle_contour = max(middle_contours, key=cv2.contourArea)

    # 4. Simplify the outline to a few straight sides
    perimeter = cv2.arcLength(middle_contour, True)            # True = closed shape
    simplified = cv2.approxPolyDP(middle_contour,
                                  CORNER_SIMPLIFY_FRACTION * perimeter, True)

    # 5. A board's middle should simplify to exactly 4 corners
    if len(simplified) != 4:
        return None
    return simplified.reshape(4, 2)                            # from (4, 1, 2) to (4, 2)


def order_corners(corners):
    """
    Puts 4 corner points in a fixed order: top-left, top-right, bottom-right, bottom-left.

    Input:  4 x 2 array of (x, y) points, in any order.
    Output: 4 x 2 array in the order [top-left, top-right, bottom-right, bottom-left].

    How: in images, x grows to the right and y grows DOWN.
      x + y is smallest at the top-left and biggest at the bottom-right.
      y - x is smallest at the top-right and biggest at the bottom-left.
    """
    corner_sums = corners[:, 0] + corners[:, 1]           # x + y for each corner
    corner_differences = corners[:, 1] - corners[:, 0]    # y - x for each corner

    top_left = corners[np.argmin(corner_sums)]            # argmin = position of the smallest value
    bottom_right = corners[np.argmax(corner_sums)]        # argmax = position of the biggest value
    top_right = corners[np.argmin(corner_differences)]
    bottom_left = corners[np.argmax(corner_differences)]

    ordered_corners = np.array([top_left, top_right, bottom_right, bottom_left])
    return ordered_corners


def draw_and_print_contours(photo_bgr, contours, blue_mask, file_name):
    """
    Draws every contour bigger than MIN_AREA_TO_SHOW on a copy of the photo,
    labels it with a number, and prints its area, blue fraction and bounding box.
    Green outline = looks like a board. Red outline = rejected.
    Output: the photo with the drawings on it.
    """
    drawing = photo_bgr.copy()
    print('--- ' + file_name + ' ---')

    contour_number = 0
    for contour in contours:
        area = cv2.contourArea(contour)            # pixels inside the outline
        if area < MIN_AREA_TO_SHOW:
            continue

        # Smallest upright rectangle around the blob: top-left corner (x, y), width, height
        x, y, width, height = cv2.boundingRect(contour)

        # Board check: a real board has a grey middle, so it is only partly blue
        blue_fraction = measure_blue_fraction(contour, blue_mask)
        if blue_fraction <= MAX_BLUE_FRACTION:
            verdict = 'BOARD'
            outline_colour = (0, 255, 0)      # green (OpenCV colours are B, G, R)
        else:
            verdict = 'rejected (solid blue)'
            outline_colour = (0, 0, 255)      # red

        print('  #' + str(contour_number) + ': area ' + str(int(area)) +
              ', blue fraction ' + str(round(blue_fraction, 2)) +
              ', box x=' + str(x) + ' y=' + str(y) +
              ' w=' + str(width) + ' h=' + str(height) +
              '  -> ' + verdict)

        # For boards: find the 4 corners of the grey middle and draw them as yellow dots
        if verdict == 'BOARD':
            corners = find_grey_middle_corners(contour, blue_mask)
            if corners is None:
                print('     corners: NOT FOUND (not 4 corners)')
            else:
                ordered_corners = order_corners(corners)
                print('     corners (TL, TR, BR, BL): ' + str(ordered_corners.tolist()))
                corner_labels = ['TL', 'TR', 'BR', 'BL']
                for corner_index in range(4):
                    corner_x = int(ordered_corners[corner_index][0])
                    corner_y = int(ordered_corners[corner_index][1])
                    cv2.circle(drawing, (corner_x, corner_y), 6, (0, 255, 255), -1)
                    cv2.putText(drawing, corner_labels[corner_index],
                                (corner_x + 8, corner_y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Outline in the verdict colour, thickness 2, and the number just above the blob
        cv2.drawContours(drawing, [contour], -1, outline_colour, 2)
        cv2.putText(drawing, '#' + str(contour_number), (x, max(y - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, outline_colour, 2)
        contour_number = contour_number + 1

    cv2.putText(drawing, file_name, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    return drawing


def main():
    # 1. Find the photos
    if len(sys.argv) > 1:
        folder = os.path.expanduser(sys.argv[1])
    else:
        folder = os.path.expanduser('~/sign_frames')
    photo_paths = sorted(glob.glob(os.path.join(folder, '*.png')))
    if len(photo_paths) == 0:
        print('No .png files found in ' + folder)
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    photo_index = 0
    show_new_photo = True

    while True:
        # 2. Only redo the work when we move to a different photo
        if show_new_photo:
            photo_path = photo_paths[photo_index]
            photo_bgr = cv2.imread(photo_path)
            contours, blue_mask = find_blue_contours(photo_bgr)
            drawing = draw_and_print_contours(photo_bgr, contours, blue_mask,
                                              os.path.basename(photo_path))
            cv2.imshow(WINDOW_NAME, drawing)
            show_new_photo = False

        # 3. Keys
        key = cv2.waitKey(50) & 0xFF
        if key == ord('n'):
            photo_index = (photo_index + 1) % len(photo_paths)
            show_new_photo = True
        elif key == ord('p'):
            photo_index = (photo_index - 1) % len(photo_paths)
            show_new_photo = True
        elif key == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()