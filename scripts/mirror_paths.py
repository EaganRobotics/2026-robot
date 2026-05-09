#!/usr/bin/env python3
"""Mirror Kansas Left Fast overcross paths to Kansas Right Fast overcross."""

import json
import math

FIELD_Y_CENTER = 4.035  # Half of field Y size 8.07m

def mirror_y(y):
    """Mirror Y coordinate across field center."""
    return FIELD_Y_CENTER - (y - FIELD_Y_CENTER)

def mirror_rotation(deg):
    """Mirror a rotation angle (negate it)."""
    # Normalize to -180..180
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return -deg

def mirror_point(p):
    """Mirror a point {'x': x, 'y': y}."""
    return {"x": p["x"], "y": mirror_y(p["y"])}

def mirror_path(input_path):
    """Mirror a pathplanner path from left to right."""
    with open(input_path, 'r') as f:
        path = json.load(f)
    
    # Mirror waypoints
    for wp in path["waypoints"]:
        if wp["anchor"]:
            wp["anchor"] = mirror_point(wp["anchor"])
        if wp["prevControl"]:
            wp["prevControl"] = mirror_point(wp["prevControl"])
        if wp["nextControl"]:
            wp["nextControl"] = mirror_point(wp["nextControl"])
    
    # Mirror rotation targets
    for rt in path["rotationTargets"]:
        rt["rotationDegrees"] = mirror_rotation(rt["rotationDegrees"])
    
    # Mirror goal end state rotation
    if "goalEndState" in path:
        path["goalEndState"]["rotation"] = mirror_rotation(path["goalEndState"]["rotation"])
    
    # Mirror ideal starting state rotation
    if "idealStartingState" in path:
        path["idealStartingState"]["rotation"] = mirror_rotation(path["idealStartingState"]["rotation"])
    
    return path

def main():
    # Mirror KS.L.1.oc -> KS.R.1.oc
    left_path1 = "src/main/deploy/pathplanner/paths/KS.L.1.oc.path"
    right_path1 = "src/main/deploy/pathplanner/paths/KS.R.1.oc.path"
    mirrored1 = mirror_path(left_path1)
    with open(right_path1, 'w') as f:
        json.dump(mirrored1, f, indent=2)
    print(f"Created {right_path1}")
    
    # Mirror KS.L.2.f.oc -> KS.R.2.f.oc
    left_path2 = "src/main/deploy/pathplanner/paths/KS.L.2.f.oc.path"
    right_path2 = "src/main/deploy/pathplanner/paths/KS.R.2.f.oc.path"
    mirrored2 = mirror_path(left_path2)
    with open(right_path2, 'w') as f:
        json.dump(mirrored2, f, indent=2)
    print(f"Created {right_path2}")

if __name__ == "__main__":
    main()
