# Coverage Path Planning — Learning Repo

Educational project for learning coverage path planning, built in layers.

The map goes through three stages:

    OccupancyGrid  →  MapGeometry  →  BCD
    (raw grid)        (contours /      (Boustrophedon
                       polygons)        Cellular Decomposition)

1. **OccupancyGrid** — raster grid of Free / Occupied / Unknown cells,
   loaded from an image.
2. **MapGeometry** — traces the free-space boundaries into contours
   (outer border + holes) using Moore-neighbor tracing.
3. **BCD** — Boustrophedon Cellular Decomposition. *(in progress)*
   Splits free space into cells that can each be swept back and forth.