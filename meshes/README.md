# Meshes

3D assets shipped with `l2lidar_node` for URDF visualization and physical integration.

## `l2_lidar.stl`

Binary STL of the Unitree L2 device itself.

- **Source**: derived from Unitree's official STEP file (published on their support download site), imported into Fusion 360, exported as binary STL at Medium refinement.
- **Geometry**: ~9,700 triangles, ~486 KB. Bounding box ~75 × 75 × 64 mm.
- **Coordinate frame**: mesh origin sits at the geometric center of the bottom mounting surface (z = 0). Mesh extends from z ≈ 0 to z ≈ 64 mm. This matches Unitree's stated cloud-frame convention exactly — point a URDF `<link>`'s origin at where the L2's mounting surface should be, attach this mesh, and the visual is geometrically correct relative to the published cloud data.
- **Use**: attach as a `<visual>` mesh on the URDF link you've assigned to `cloud_frame`. See `urdf/examples/` for working examples.

## `l2_damping_pad.stl` + `l2_damping_pad.svg`

TPU vibration-damping pad for use between the L2 and a robot's mounting surface.

- **Source**: designed from scratch.
- **Dimensions**: Ø79 mm × 1.5 mm, with the L2's bolt-circle pattern (Ø51 mm circle, square inscribed, rotated 22.5° from cable-exit axis). The 4× Ø3.5 mm holes line up with the L2's M3 mounting holes.
- **Intended material**: TPU 95A or 75A. Slice with 0.2 mm layer height, walls 1.0 mm, infill 100%.
- **STL**: for 3D printing. `~1 MB`.
- **SVG**: for laser-cut alternatives (e.g., 1.5 mm cork or rubber sheet).
- **Use**: sandwich between the L2 base plate and your robot's mounting plate. Helps decouple L2 motor vibration from the host structure, reducing the likelihood of vibration-induced RTK GPS FIX → FLOAT transitions or IMU drift artifacts on platforms with mast-mounted sensors.

## Licensing

The L2 device STL is a format conversion of Unitree's published CAD; Unitree's intent in publishing the STEP is to enable integration work. The damping pad is original work; treat both as freely usable under the same GPL-3.0 terms as the rest of the package unless your downstream constraints require otherwise.
