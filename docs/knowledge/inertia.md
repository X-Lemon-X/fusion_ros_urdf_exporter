# Inertia (`InertiaCalculator`)

Each link's `<inertial>` comes from Fusion's physical properties of the
top-level occurrence, subcomponents included. Units as in
[frames-and-units.md](frames-and-units.md#units).

## Computation

1. `occurrence.getPhysicalProperties(inertia_accuracy)`. If that call fails
   it falls back to `occurrence.physicalProperties`.
2. Mass ≤ 0 or `None` → warning "Mass is 0 …" and no `<inertial>` for the
   link.
3. COM: `props.centerOfMass` (world, cm) → m.
4. Tensor: `props.getXYZMomentsOfInertia()` returns
   `(ok, xx, yy, zz, xy, yz, xz)` in kg·cm² → kg·m². It is used as
   `[[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]`, i.e. **about the world
   origin, with the off-diagonal terms used as returned** (no sign flip).
   This is the convention fusion2urdf uses. Not verified, see Q3 in
   [open-questions.md](../open-questions.md).
5. Parallel-axis theorem to the COM: `I_com = I_origin − m·D(com)`, still in
   world axes.
6. Rotate into link axes: `I_link = Rᵀ · I_com · R`, with `R` the link's
   world rotation. COM in the link frame: `inv(T_link_world) · com_world`.
7. URDF: `<origin xyz="COM" rpy="0 0 0">`, `<mass>`, `<inertia ixx … izz>`.

## Checks

`InertiaCalculator.validate()` adds a warning (the link is still exported) if:
- any diagonal term is ≤ 0, or
- the triangle inequality fails (`ixx + iyy < izz`, and the two other
  combinations, tolerance 1e-12).

## Known limitations (as of 2026-11-06)

- The masses are only as good as the materials assigned in Fusion.
- The off-diagonal sign convention is an assumption. On rft_robodog the
  off-diagonal terms are close to zero, so a wrong sign would matter little
  there.
