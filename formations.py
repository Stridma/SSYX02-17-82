"""
Function that returns points in space that represents the desired formation given as a string. The list is now done
manually, but could be extended to handle any number of robots and "any" formation.
"""


def set_formation(formation):
    formation = formation.lower()
    formations = {"two": [[3, 0], [-3, 0]],
                  "triangle": [[0, 0], [0, 1], [0.71, 0.5]],
                  "line": [[0, -4], [0, 0], [0, 4]]}

    # If formation not found, tell user
    return formations.get(formation, 'Formation not found.')
