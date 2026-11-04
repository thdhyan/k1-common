# k1-common

Robot contract: joints, gains, obs/action layout, wiring, motion-clip schema.

| | |
|---|---|
| Layer | **L0**: Contract (no heavy deps) |
| May import | nothing |
| Files / LOC | 11 files · 719 lines (py, sh, cpp) |
| Part of | [k1-workspace](https://github.com/thdhyan/k1-workspace) (architecture and layering rules live there) |

## Where it sits

```mermaid
flowchart TB
  k1_utils[k1-utils]
  k1_runtime[k1-runtime]
  k1_rl_isaac[k1-rl-isaac]
  k1_rl_mj[k1-rl-mj]
  k1_mimic[k1-mimic]
  k1_teleop[k1-teleop]
  k1_perception[k1-perception]
  k1_vla[k1-vla]
  k1_data[k1-data]
  k1_deploy_robot[k1-deploy-robot]
  k1_deploy_laptop[k1-deploy-laptop]
  k1_common((k1-common))
  style k1_common fill:#e8833a,color:#000
  k1_utils --> k1_common
  k1_runtime --> k1_common
  k1_rl_isaac --> k1_common
  k1_rl_mj --> k1_common
  k1_mimic --> k1_common
  k1_teleop --> k1_common
  k1_perception --> k1_common
  k1_vla --> k1_common
  k1_data --> k1_common
  k1_deploy_robot --> k1_common
  k1_deploy_laptop --> k1_common
```

Used by: k1-utils, k1-runtime, k1-rl-isaac, k1-rl-mj, k1-mimic, k1-teleop, k1-perception, k1-vla, k1-data, k1-deploy-robot, k1-deploy-laptop.

![k1-workspace layers](https://raw.githubusercontent.com/thdhyan/k1-workspace/main/docs/img/architecture.png)

## Install

```bash
uv add git+https://github.com/thdhyan/k1-common
# or, from a checkout
uv sync
```

## Layout

```
README.md
pyproject.toml
ros/  (8 files)
  k1_interfaces/  (8 files)
src/  (4 files)
  k1_common/  (4 files)
tests/  (1 files)
  test_k1_wiring.py
```

## Dependencies

- Declared sibling libraries: none (see `pyproject.toml`, `[tool.uv.sources]`).
- PyPI: mujoco, numpy, pytest.
- Provided by the environment (ROS 2, Isaac, mjlab, robot SDK) or unresolved: `mjlab`, `pxr`.

## Hard-coded paths into other repos

These files still name paths that now live in another repo (hard-coded path strings, not imports). Pass them as arguments / config when you touch the file:

| File | Refers to | Owner |
|---|---|---|
| `src/k1_common/k1_wiring.py` | `real_robot/` | k1-data |

## Origin

Extracted from `booster_ws` (branch `dev/push-training-wip`, working tree of 2026-11-03) by
`tools/build_workspace.py` in k1-workspace. Old path to new path for every file: [`docs/migration_map.tsv`](https://github.com/thdhyan/k1-workspace/blob/main/docs/migration_map.tsv) (filter on `k1-common`).
History was not carried over: the working tree was copied as is, including previously untracked files.
