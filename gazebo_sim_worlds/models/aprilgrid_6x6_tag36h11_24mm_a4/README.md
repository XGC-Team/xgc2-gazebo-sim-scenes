# A4 6×6 AprilGrid

Gazebo representation of board profile `a4_6x6_24mm_30pct_kalibr_v1`:

- 6×6 `tag36h11`, IDs 0–35;
- tag edge 0.024 m;
- clear gap 0.0072 m (30%);
- active calibration span 0.1800 m;
- rendered symmetric-target span 0.1944 m.

The retained texture is the canonical Kalibr datum: ID 0 is lower-left, IDs
increase along +X then +Y, and every tag is OpenCV marker rotation 2. The model
name and texture filename describe geometry and remain stable; they are not a
runtime alias for the retired profile identifier.

It is a first-class calibration profile and uses the same detector, solver,
persistence and camera workflow as the 88 mm board.
