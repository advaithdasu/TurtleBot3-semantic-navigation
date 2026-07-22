# Vendored + authored Gazebo models

## Provenance

The `aws_robomaker_*` directories are vendored unmodified (except URI fixes,
see below) from the AWS Robotics sample worlds, `ros2` branches, both
licensed **MIT-0** (see `LICENSE` in this directory):

- [aws-robotics/aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) (`ros2` branch)
- [aws-robotics/aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world) (`ros2` branch)

Local changes to the vendored copies:

- Rewrote the warehouse models' `file://models/<name>/...` mesh URIs to
  portable `model://<name>/...` URIs (resolved via `GAZEBO_MODEL_PATH`).
- Deleted `.DS_Store` / `.psd` files.

### Vendored directories

| Directory | Source repo | Footprint (collision, m) |
|---|---|---|
| `aws_robomaker_warehouse_ShelfF_01` | small-warehouse-world | 2.10 x 18.05 x 6.53 (full-length rack; too large for the 8x6 semantic world) |
| `aws_robomaker_warehouse_ShelfD_01` | small-warehouse-world | 3.92 x 0.88 x 2.61 (used as the north-wall shelf in `warehouse_aws_semantic.world`) |
| `aws_robomaker_warehouse_ClutteringC_01` | small-warehouse-world | 1.77 x 2.06 x 1.79 |
| `aws_robomaker_warehouse_PalletJackB_01` | small-warehouse-world | 1.16 x 0.54 x 0.98 |
| `aws_robomaker_warehouse_Bucket_01` | small-warehouse-world | 0.94 x 1.22 x 1.41 |
| `aws_robomaker_warehouse_TrashCanC_01` | small-warehouse-world | 1.48 x 0.91 x 1.31 |
| `aws_robomaker_residential_ChairA_01` | small-house-world | — |
| `aws_robomaker_residential_ChairD_01` | small-house-world | 0.58 x 0.51 x 0.78 |
| `aws_robomaker_residential_SofaB_01` | small-house-world | 0.74 x 0.85 x 0.91 (seat surface at z = 0.44) |

## Authored here (not vendored)

Color-variant models for HSV-based visual grounding — an SDF `<material>`
with flat ambient/diffuse replaces the mesh texture in Gazebo Classic, which
is intentional: colors must be unambiguous.

| Directory | Basis | Color (diffuse) |
|---|---|---|
| `chair_blue` | ChairD_01 meshes | 0 0 1 1 |
| `chair_red` | ChairD_01 meshes | 1 0 0 1 |
| `sofa_orange` | SofaB_01 meshes | 1 0.45 0 1 |
| `box_blue` | pure-SDF 0.4 x 0.4 x 0.35 cuboid | 0 0 1 1 |
| `box_red` | pure-SDF 0.4 x 0.4 x 0.35 cuboid | 1 0 0 1 |
| `box_yellow` | pure-SDF 0.4 x 0.4 x 0.35 cuboid | 1 0.9 0 1 |
