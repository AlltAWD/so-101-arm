# SO-101 link measurements

**Superseded as the FK source.** The exact link geometry comes from the official
CAD-generated URDF, recorded in `kinematics/urdf_params.py` (upper arm ≈ 116 mm,
forearm ≈ 135 mm, wrist ≈ 64 mm, base height ≈ 62 mm). No hand-measuring needed
to *build* FK.

The ruler still has a job: **validating FK output.** After FK predicts the
fingertip (x, y, z) for a commanded pose, measure the real fingertip position and
compare. Agreement to ~1 cm = FK is correct. Optionally cross-check one link
length by hand as a sanity check on the URDF.
