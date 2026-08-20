# Field-matched AprilGrid

This model reproduces the station's physical camera-calibration target using
the Kalibr AprilGrid convention:

- 6 columns by 6 rows, IDs 0 through 35
- `tag36h11`
- 88 mm tag edge
- 30% spacing, or 26.4 mm between tag edges
- 660.0 mm calibration datum from the first tag edge to the last tag edge

Kalibr's symmetric black corner marks occupy the gaps and extend 26.4 mm past
the outer tag edges. The Gazebo texture therefore covers 712.8 x 712.8 mm,
centred on the same 660.0 mm calibration datum. The backing adds a 10 mm white
surround and is not part of the calibrated geometry.

`source/aprilgrid_6x6_tag36h11_88mm_30pct.pdf` was exported with Kalibr's
official generator at the pinned revision recorded in `source_manifest.json`:

```bash
kalibr_create_target_pdf aprilgrid_6x6_tag36h11_88mm_30pct \
  --type apriltag --nx 6 --ny 6 --tsize 0.088 --tspace 0.3 \
  --tfam t36h11
```

The texture is the target-only square from that vector PDF at 5 pixels/mm.
Axes and caption are deliberately excluded because they are outside the target.
Keep nearest physical print scale at 100%; do not use "fit to page".
