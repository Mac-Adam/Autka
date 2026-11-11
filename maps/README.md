# Saved maps

Save a completed occupancy map here while the simulation and Cartographer are
running. From this package directory, run:

```bash
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/track"
```

This creates `track.yaml` and `track.pgm`. The YAML file stores the map
resolution, origin, and occupancy thresholds; the PGM image stores the cell
occupancy values, including unknown cells. Keep both files together.