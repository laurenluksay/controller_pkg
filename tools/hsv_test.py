"""
hsv_test.py - tool for tuning the blue sign-border colour filter.

Not a ROS node. It loads the saved sign photos, keeps only the pixels
inside an HSV range, and shows the photo next to the resulting mask.
Six sliders let you change the range and see the effect straight away.

Run it:
    python3 hsv_test.py ~/sign_frames

Keys (click on the image window first):
    n = next photo
    p = previous photo
    q = quit (prints the final slider values)
"""

import glob
import os
import sys

import cv2
import numpy as np


# Starting values, measured from the 7 saved photos (2026-11-27).
# OpenCV HSV ranges: hue 0-179, saturation 0-255, value 0-255.
START_HUE_MIN = 110
START_HUE_MAX = 130
START_SAT_MIN = 90     # sign border >= 126, water <= 60
START_SAT_MAX = 255
START_VAL_MIN = 40
START_VAL_MAX = 255

WINDOW_NAME = 'HSV tuning: photo (left) and mask (right)'


def do_nothing(value):
    # OpenCV needs a function to call when a slider moves.
    # We read the sliders ourselves in the loop, so this does nothing.
    pass


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
    print('Found ' + str(len(photo_paths)) + ' photos in ' + folder)

    # 2. Make the window and its six sliders
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Hue min', WINDOW_NAME, START_HUE_MIN, 179, do_nothing)
    cv2.createTrackbar('Hue max', WINDOW_NAME, START_HUE_MAX, 179, do_nothing)
    cv2.createTrackbar('Sat min', WINDOW_NAME, START_SAT_MIN, 255, do_nothing)
    cv2.createTrackbar('Sat max', WINDOW_NAME, START_SAT_MAX, 255, do_nothing)
    cv2.createTrackbar('Val min', WINDOW_NAME, START_VAL_MIN, 255, do_nothing)
    cv2.createTrackbar('Val max', WINDOW_NAME, START_VAL_MAX, 255, do_nothing)

    photo_index = 0

    while True:
        # 3. Load the current photo and convert it to HSV
        photo_path = photo_paths[photo_index]
        photo_bgr = cv2.imread(photo_path)                       # OpenCV loads colours as BGR
        photo_hsv = cv2.cvtColor(photo_bgr, cv2.COLOR_BGR2HSV)

        # 4. Read the six sliders
        hue_min = cv2.getTrackbarPos('Hue min', WINDOW_NAME)
        hue_max = cv2.getTrackbarPos('Hue max', WINDOW_NAME)
        sat_min = cv2.getTrackbarPos('Sat min', WINDOW_NAME)
        sat_max = cv2.getTrackbarPos('Sat max', WINDOW_NAME)
        val_min = cv2.getTrackbarPos('Val min', WINDOW_NAME)
        val_max = cv2.getTrackbarPos('Val max', WINDOW_NAME)

        lower_bound = np.array([hue_min, sat_min, val_min])
        upper_bound = np.array([hue_max, sat_max, val_max])

        # 5. Make the mask: white where the pixel is inside the range, black elsewhere
        blue_mask = cv2.inRange(photo_hsv, lower_bound, upper_bound)

        # 6. Show the photo and the mask side by side, with the file name on top
        mask_in_colour = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)   # so both halves have 3 channels
        side_by_side = np.hstack([photo_bgr, mask_in_colour])
        file_name = os.path.basename(photo_path)
        cv2.putText(side_by_side, file_name, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.imshow(WINDOW_NAME, side_by_side)

        # 7. Handle key presses (waitKey also lets the window redraw)
        key = cv2.waitKey(50) & 0xFF
        if key == ord('n'):
            photo_index = (photo_index + 1) % len(photo_paths)
        elif key == ord('p'):
            photo_index = (photo_index - 1) % len(photo_paths)
        elif key == ord('q'):
            break

    # 8. Print the final values so you can copy them into the detector
    print('Final range:')
    print('  lower = [' + str(hue_min) + ', ' + str(sat_min) + ', ' + str(val_min) + ']')
    print('  upper = [' + str(hue_max) + ', ' + str(sat_max) + ', ' + str(val_max) + ']')
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()