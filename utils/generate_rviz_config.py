#!/usr/bin/env python3
"""Generate an RViz2 config by filling Camera displays into rviz2_config_base.rviz."""

import argparse
import os
import sys

BASE_TEMPLATE = os.path.join(
    os.path.dirname(__file__),
    "..", "config_ground_station", "rviz2_config_base.rviz",
)

CAMERA_DISPLAY = """\
    - Class: rviz_default_plugins/Camera
      Enabled: true
      Far Plane Distance: 1000
      Image Rendering: background and overlay
      Name: Camera {drone}
      Overlay Alpha: 0.5
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /{drone}/sensor_measurements/image_with_markers
      Value: true
      Visibility:
        Grid: true
        HistoryPoses: true
        Marker: true
        Path: true
        ReferencePose: true
        RobotModel: true
        SolarPanels: true
        TF: true
        Value: true
      Zoom Factor: 1
"""


def generate(drones: list[str]) -> str:
    with open(BASE_TEMPLATE) as f:
        template = f.read()

    panel_expanded = "".join(f"        - /Camera {d}1\n" for d in drones)
    camera_displays = "".join(CAMERA_DISPLAY.format(drone=d) for d in drones)
    camera_geometry = "".join(f"  Camera {d}:\n    collapsed: false\n" for d in drones)

    return (
        template
        .replace("__CAMERAS_PANEL_EXPANDED__", panel_expanded)
        .replace("__CAMERA_DISPLAYS__", camera_displays)
        .replace("__CAMERA_GEOMETRY__", camera_geometry)
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-n", "--namespaces", required=True,
                        help="Comma-separated drone namespaces, e.g. drone0,drone1")
    parser.add_argument("-o", "--output", default="-",
                        help="Output file path (default: stdout)")
    args = parser.parse_args()

    drones = [d.strip() for d in args.namespaces.split(",") if d.strip()]
    config = generate(drones)

    if args.output == "-":
        sys.stdout.write(config)
    else:
        with open(args.output, "w") as f:
            f.write(config)
        print(f"[generate_rviz_config] Written {len(drones)} drone(s) to {args.output}", flush=True)


if __name__ == "__main__":
    main()
