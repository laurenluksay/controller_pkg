"""
place_signs.py - TESTING TOOL ONLY (not part of the robot)

Moves the 7 route signs into a fixed test layout to test extremes of
sign variation.

Usage (sim running, workspace sourced, from the controller_pkg repo folder):
    python3 tools/place_signs.py A
"""

import sys

from controller_pkg.driving import teleport

# ----------- 1. Fixed Sign Info -----------
# name: (base_x, base_y, z, base_yaw)
# Moving signs use the middle of their random range as base value.
# z values from environment
SIGN_INFO = {
    'sign_0': (-5.81, -1.64,  0.04, 3.14),   # clue 1 SIZE
    'sign_1': (-5.16,  1.355, 0.04, 3.14),   # clue 2 VICTIM  (y moves 1.11 to 1.60)
    'sign_2': (-4.0,   1.67,  0.04, 1.57),   # clue 3 CRIME
    'sign_3': (-0.83,  0.315, 0.04, 0.0),    # clue 4 TIME    (y moves 0.00 to 0.63)
    'sign_4': (-0.83, -1.64,  0.04, 3.14),   # clue 5 PLACE   (y moves -1.84 to -1.44)
    'sign_5': ( 3.205, -1.71, 0.04, 1.57),   # clue 6 MOTIVE  (x moves 3.00 to 3.41)
    'sign_6': ( 3.8,   2.01,  0.04, -1.57),  # clue 7 WEAPON
}

# ---------- 2. Test layouts ----------
# 'moves': sign name -> (axis to change, new value). Signs not listed stay at base.
# 'yaw_offset': added to EVERY sign's base yaw (random turn is +/- 0.15 rad).
SCENARIOS = {
    'A': {'moves': {'sign_1': ('y', 1.11), 'sign_3': ('y', 0.00),
                    'sign_4': ('y', -1.84), 'sign_5': ('x', 3.00)},
          'yaw_offset': 0.0},     # all low
    'B': {'moves': {'sign_1': ('y', 1.60), 'sign_3': ('y', 0.63),
                    'sign_4': ('y', -1.44), 'sign_5': ('x', 3.41)},
          'yaw_offset': 0.0},     # all high
    'C': {'moves': {'sign_1': ('y', 1.57)},
          'yaw_offset': 0.0},     # clue 2 just past the SIGN3 stop
    'D': {'moves': {'sign_1': ('y', 1.11), 'sign_3': ('y', 0.00),
                    'sign_4': ('y', -1.84), 'sign_5': ('x', 3.00)},
          'yaw_offset': -0.15},   # all low, turned one way
    'E': {'moves': {'sign_1': ('y', 1.60), 'sign_3': ('y', 0.63),
                    'sign_4': ('y', -1.44), 'sign_5': ('x', 3.41)},
          'yaw_offset': 0.15},    # all high, turned the other way
    'M': {'moves': {},
          'yaw_offset': 0.0},     # everything at the middle (reset)
}


# ---------- 3. Main part (YOU WRITE THIS) ----------
def main():
    # Step 1: check a letter was given and is in SCENARIOS.
    #         If not: print the valid letters and return.

    # Step 2: get this scenario's 'moves' dict and 'yaw_offset'.

    # Step 3: loop over every sign in SIGN_INFO (all 7, every time):
    #           - start from its base x, y, z, yaw
    #           - if this sign is in 'moves', replace x or y with the new value
    #           - add yaw_offset to yaw
    #           - call teleport(...) with model_name set to the sign's name
    #           - print the sign name, where it went, and True/False
    if len(sys.argv) < 2:
        print('Usage: python3 tools/place_signs.py <letter>')
        print('Valid letters: ' + ', '.join(SCENARIOS.keys()))
        return

    letter = sys.argv[1].upper()

    if letter not in SCENARIOS:
        print('Unknown layout ' + letter + '. Valid letters: ' + ', '.join(SCENARIOS.keys()))
        return

    scenario = SCENARIOS[letter]
    moves = scenario['moves']
    yaw_offset = scenario['yaw_offset']

    print('Placing signs for layout ' + letter)

    for sign_name, (x, y, z, yaw) in SIGN_INFO.items():
        if sign_name in moves:
            axis, new_value = moves[sign_name]
            if axis == 'x':
                x = new_value
            else:
                y = new_value

        yaw = yaw + yaw_offset

        moved = teleport(x, y, z, yaw, model_name=sign_name)

        clue_number = int(sign_name.split('_')[1]) + 1
        print(sign_name + ' (clue ' + str(clue_number) + '): '
              + 'x=' + str(round(x, 2)) + ' y=' + str(round(y, 2))
              + ' yaw=' + str(round(yaw, 2)) + '  ->  ' + str(moved))




if __name__ == '__main__':
    main()