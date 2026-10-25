# Third-party code in this repository

These folders are **vendored copies** of upstream projects. They are kept unmodified
except where stated, so that a fresh clone builds offline at exactly the tested versions.

| Folder | Upstream | Version | License | Local changes |
|---|---|---|---|---|
| `unitree_sdk2/` | https://github.com/unitreerobotics/unitree_sdk2 | commit `d1abf001220d3e5f59e00a0c4aaa595e00089a4a` (2026-10-15) | BSD-3-Clause (`unitree_sdk2/LICENSE`); bundled CycloneDDS / iceoryx under `unitree_sdk2/licenses/` | none |
| `unitree_mujoco/` | https://github.com/unitreerobotics/unitree_mujoco | commit `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` (2026-09-07) | BSD-3-Clause (`unitree_mujoco/LICENSE`) | **added** `simulate/nav_ext/` (scanner simulator) and `unitree_robots/go2/scene_nav.xml`; `simulate/mujoco` is a symlink to `../../third_party/mujoco-3.3.6` |
| `third_party/mujoco-3.3.6/` | https://github.com/google-deepmind/mujoco/releases/tag/3.3.6 | 3.3.6, `mujoco-3.3.6-linux-x86_64.tar.gz` | Apache-2.0 (`third_party/mujoco-3.3.6/LICENSE`, plus `THIRD_PARTY_NOTICES` for its bundled libraries) | added the `LICENSE` file from the MuJoCo 3.3.6 source tag (the binary archive does not ship it) |

The MuJoCo binaries are for **Linux x86_64**. For another architecture (for example an aarch64 Jetson), replace the
folder with the matching MuJoCo 3.3.6 release; the `simulate/mujoco` symlink stays the same.

## Updating a vendored dependency
1. Replace the folder contents with the new upstream version. Keep our two additions in `unitree_mujoco/`.
2. Update the table above.
3. `bash scripts/build.sh`, then run `bash scripts/run_acceptance_tests.sh` against a fresh `bash scripts/run_sim.sh`.
