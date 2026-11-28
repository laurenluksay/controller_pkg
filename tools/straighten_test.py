"""
straighten_test.py - checks straighten_board on the saved sign photos.

Not part of the robot. For each saved photo it:
  1. finds the board with detect_board
  2. straightens it with straighten_board
  3. shows the photo (with the 4 corners drawn) next to the straightened board
  4. saves that side-by-side picture into ~/sign_frames/straightened/

Run from the controller_pkg repo folder:
    python3 tools/straighten_test.py
Any key = next photo, q = quit.
"""

import os
import sys

import cv2
import numpy as np

# Let Python find the controller_pkg module when this file is run from tools/
# (adds the repo folder, one level up from this file, to the import search path).
REPO_FOLDER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_FOLDER)

from controller_pkg.clue_detection import (detect_board, straighten_board,
                                           BLUE_LOWER, BLUE_UPPER)


PHOTO_FOLDER = os.path.expanduser('~/sign_frames')
OUTPUT_FOLDER = os.path.join(PHOTO_FOLDER, 'straightened')

# One photo per sign, from the camera that reads it. Marnu's photo is optional.
PHOTO_NAMES = [
    'sign1_front.png',
    'sign2_front.png',
    'sign3_right.png',
    'sign4_front.png',
    'sign5_back.png',
    'sign6_left.png',
    'sign7_front.png',
    'sign4_front_marnu.png',
]

# Same minimum board area the live node uses for every camera.
MIN_BOARD_AREA = 3000

CORNER_LABELS = ['TL', 'TR', 'BR', 'BL']


def draw_corners(photo_bgr, ordered_corners):
    """
    Input:  a photo and its 4 ordered corners.
    Output: a copy of the photo with the grey middle outlined in green and
            the corners as yellow dots labelled TL / TR / BR / BL.
    """
    drawing = photo_bgr.copy()
    cv2.polylines(drawing, [ordered_corners.reshape(-1, 1, 2)], True,
                  (0, 255, 0), 2)
    for corner_index in range(4):
        corner_x = int(ordered_corners[corner_index][0])
        corner_y = int(ordered_corners[corner_index][1])
        cv2.circle(drawing, (corner_x, corner_y), 5, (0, 255, 255), -1)
        cv2.putText(drawing, CORNER_LABELS[corner_index],
                    (corner_x + 6, corner_y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)
    return drawing


def place_side_by_side(left_image, right_image):
    """
    Input:  two BGR images (can be different heights).
    Output: one image with left_image on the left and right_image on the right;
            the shorter one is padded with black at the bottom.
    """
    tallest_height = max(left_image.shape[0], right_image.shape[0])

    left_padding = tallest_height - left_image.shape[0]
    left_padded = cv2.copyMakeBorder(left_image, 0, left_padding, 0, 0,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))

    right_padding = tallest_height - right_image.shape[0]
    right_padded = cv2.copyMakeBorder(right_image, 0, right_padding, 0, 0,
                                      cv2.BORDER_CONSTANT, value=(0, 0, 0))

    return np.hstack([left_padded, right_padded])


def count_blue_pixels(image_bgr):
    """
    Input:  a BGR image.
    Output: number of pixels in the blue border colour range (should be ~0 in a
            straightened board, since the blue frame should be cropped away).
    """
    image_hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    blue_mask = cv2.inRange(image_hsv, BLUE_LOWER, BLUE_UPPER)
    return cv2.countNonZero(blue_mask)


def main():
    os.makedirs(OUTPUT_FOLDER, exist_ok=True)

    for photo_name in PHOTO_NAMES:
        photo_path = os.path.join(PHOTO_FOLDER, photo_name)
        if not os.path.exists(photo_path):
            print(f'{photo_name}: not found, skipping')
            continue

        photo_bgr = cv2.imread(photo_path)

        # 1. Find the board
        ordered_corners, board_contour = detect_board(photo_bgr, MIN_BOARD_AREA)
        if ordered_corners is None:
            print(f'{photo_name}: NO BOARD FOUND')
            continue

        # 2. Straighten it
        straight_board = straighten_board(photo_bgr, ordered_corners)

        # 3. Print numbers to check
        blue_pixel_count = count_blue_pixels(straight_board)
        total_pixel_count = straight_board.shape[0] * straight_board.shape[1]
        blue_percent = 100.0 * blue_pixel_count / total_pixel_count
        print(f'{photo_name}: corners {ordered_corners.tolist()}, '
              f'output {straight_board.shape[1]} x {straight_board.shape[0]}, '
              f'blue left {blue_pixel_count} px ({blue_percent:.2f} %)')

        # 4. Show and save photo + straightened board side by side
        photo_with_corners = draw_corners(photo_bgr, ordered_corners)
        side_by_side = place_side_by_side(photo_with_corners, straight_board)

        output_path = os.path.join(OUTPUT_FOLDER, photo_name)
        cv2.imwrite(output_path, side_by_side)

        cv2.imshow('photo (left) and straightened board (right)', side_by_side)
        pressed_key = cv2.waitKey(0) & 0xFF
        if pressed_key == ord('q'):
            break

    cv2.destroyAllWindows()
    print(f'Side-by-side pictures saved in {OUTPUT_FOLDER}')


if __name__ == '__main__':
    main()