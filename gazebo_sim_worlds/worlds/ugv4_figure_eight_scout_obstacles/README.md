# ugv4_figure_eight_scout_obstacles

Scout Mini profile obstacle field for the four-UGV figure-eight formation
experiment.  Same ten named bodies as `ugv4_figure_eight_obstacles` (the
paper/mecanum-scale sibling), re-anchored to the Gerono eight
(A = 6 m, B/2 = 3 m, w = 0.05 rad/s -- the 2026-07-30 one-third shrink of the
earlier A = 9 / B/2 = 4.5 field, at unchanged w) and placed by a
slot-trajectory sweep against the routed x1.5 formation schedule with the
0.43 m Scout envelope:

- 4 strong single-sided bites, 0.50-0.51 m min slot clearance
  (deformation demand ~0.25 m against the 0.751 m certified requirement)
- 4 medium bodies, 0.83-0.85 m
- 2 lobe-extreme markers, 1.56-1.59 m

Pairwise surface gaps >= 2.2 m, crossing keep-out |c| - r >= 4.2 m, no
opposite-side gates within 12 s of path time.  Regenerate the world with
`tools/generate_formation_scenes.py` after editing `source_manifest.json`.

Note on the shrink: with the formation half-span (2.7 m) and the 0.03 rad/s
spin held fixed, the smaller eight is swept solid by the slot paths -- a dense
scan of the right lobe finds a maximum nearest-slot distance of 1.04 m,
attained only on the outer boundary.  The two `w_tip_*` bodies, which used to
sit inside the lobe holes at |x| = 5.9, therefore no longer have an interior
pocket to occupy and now sit in the annulus just outside the swept band near
each lobe extreme, off the x axis.  Their role is unchanged: their band is
well above the 0.751 m requirement, so they demand no deformation.
