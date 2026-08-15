# Go1 model source

The URDF and Collada meshes in this directory come from the official
[`unitreerobotics/unitree_ros`](https://github.com/unitreerobotics/unitree_ros)
repository, under `robots/go1_description`.

The clean STL files under `simplified_meshes/` come from
[`google-deepmind/mujoco_menagerie`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go1),
where they are provided as a simplified Go1 model derived from Unitree's public
URDF. Both sources provide the model under the included BSD-3-Clause license.

The generated files under `render_cache/` assemble those link meshes into a
standing-pose visual for real-time demo rendering. Physics contact continues to
use the project's simplified foot shapes.
