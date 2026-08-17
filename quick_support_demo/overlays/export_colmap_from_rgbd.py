from __future__ import annotations

import argparse
import json
import shutil
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image


CV_TO_OPENGL = np.diag([1.0, -1.0, -1.0])
PINHOLE_MODEL_ID = 1


@dataclass(frozen=True)
class ColmapImage:
    image_id: int
    name: str
    qvec: np.ndarray
    tvec: np.ndarray
    observations: tuple[tuple[float, float, int], ...]


@dataclass(frozen=True)
class SeedPoint:
    point_id: int
    xyz: np.ndarray
    rgb: np.ndarray
    image_id: int
    point2d_index: int


def depth_path_for_frame(dataset_dir: Path, frame: dict, image_name: str) -> Path:
    return dataset_dir / frame.get(
        "depth_npy_file_path",
        f"depth/{Path(image_name).stem}.npy",
    )


def rotmat_to_qvec(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to COLMAP's scalar-first quaternion."""
    rotation = np.asarray(rotation, dtype=float)
    rxx, ryx, rzx, rxy, ryy, rzy, rxz, ryz, rzz = rotation.flat
    matrix = np.array(
        [
            [rxx - ryy - rzz, 0.0, 0.0, 0.0],
            [ryx + rxy, ryy - rxx - rzz, 0.0, 0.0],
            [rzx + rxz, rzy + ryz, rzz - rxx - ryy, 0.0],
            [ryz - rzy, rzx - rxz, rxy - ryx, rxx + ryy + rzz],
        ]
    ) / 3.0
    eigenvalues, eigenvectors = np.linalg.eigh(matrix)
    qvec = eigenvectors[[3, 0, 1, 2], np.argmax(eigenvalues)]
    if qvec[0] < 0.0:
        qvec *= -1.0
    return qvec


def qvec_to_rotmat(qvec: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(qvec, dtype=float)
    return np.array(
        [
            [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * w * z, 2 * z * x + 2 * w * y],
            [2 * x * y + 2 * w * z, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * w * x],
            [2 * z * x - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x * x - 2 * y * y],
        ]
    )


def opengl_c2w_to_colmap(c2w_opengl: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert OpenGL camera-to-world to COLMAP/OpenCV world-to-camera."""
    c2w_opengl = np.asarray(c2w_opengl, dtype=float)
    rotation_c2w_cv = c2w_opengl[:3, :3] @ CV_TO_OPENGL
    rotation_w2c_cv = rotation_c2w_cv.T
    camera_center = c2w_opengl[:3, 3]
    translation_w2c_cv = -rotation_w2c_cv @ camera_center
    return rotation_w2c_cv, translation_w2c_cv


def backproject_rgbd(
    depth_m: np.ndarray,
    rgb: np.ndarray,
    c2w_opengl: np.ndarray,
    intrinsics: dict,
    stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Back-project camera-Z depth into world points and retain source pixels."""
    if stride < 1:
        raise ValueError("depth stride must be at least one")
    rows = np.arange(0, depth_m.shape[0], stride)
    cols = np.arange(0, depth_m.shape[1], stride)
    uu, vv = np.meshgrid(cols, rows)
    depth = depth_m[vv, uu]
    valid = np.isfinite(depth) & (depth > 0.0)
    z_cv = depth[valid]
    u = uu[valid].astype(float)
    v = vv[valid].astype(float)
    x_cv = (u - float(intrinsics["cx"])) * z_cv / float(intrinsics["fl_x"])
    y_cv = (v - float(intrinsics["cy"])) * z_cv / float(intrinsics["fl_y"])
    camera_cv = np.column_stack((x_cv, y_cv, z_cv))
    camera_gl = camera_cv @ CV_TO_OPENGL
    rotation_c2w_gl = np.asarray(c2w_opengl, dtype=float)[:3, :3]
    world = camera_gl @ rotation_c2w_gl.T + np.asarray(c2w_opengl, dtype=float)[:3, 3]
    colors = np.asarray(rgb, dtype=np.uint8)[vv[valid], uu[valid], :3]
    pixels = np.column_stack((u, v))
    return world.astype(np.float32), colors, pixels


def build_seed_points(
    dataset_dir: Path,
    transforms: dict,
    source_dir: Path,
    *,
    depth_stride: int,
    view_stride: int,
    voxel_size_m: float,
) -> tuple[list[SeedPoint], dict[int, list[tuple[float, float, int]]]]:
    if voxel_size_m <= 0.0:
        raise ValueError("voxel size must be positive")
    if view_stride < 1:
        raise ValueError("view stride must be at least one")

    voxels: dict[
        tuple[int, int, int],
        tuple[np.ndarray, np.ndarray, int, np.ndarray],
    ] = {}
    frames = transforms["frames"]
    for frame_index in range(0, len(frames), view_stride):
        frame = frames[frame_index]
        image_name = Path(frame["file_path"]).name
        rgb = np.asarray(Image.open(source_dir / image_name).convert("RGB"))
        depth_path = depth_path_for_frame(dataset_dir, frame, image_name)
        depth_m = np.load(depth_path)
        world, colors, pixels = backproject_rgbd(
            depth_m,
            rgb,
            np.asarray(frame["transform_matrix"], dtype=float),
            transforms,
            depth_stride,
        )
        voxel_keys = np.floor(world / voxel_size_m).astype(np.int32)
        _, first_indices = np.unique(voxel_keys, axis=0, return_index=True)
        for index in first_indices:
            key = tuple(int(value) for value in voxel_keys[index])
            if key not in voxels:
                voxels[key] = (
                    world[index],
                    colors[index],
                    frame_index + 1,
                    pixels[index],
                )

    observations: dict[int, list[tuple[float, float, int]]] = {
        image_id: [] for image_id in range(1, len(frames) + 1)
    }
    points: list[SeedPoint] = []
    for point_id, (xyz, rgb, image_id, pixel) in enumerate(voxels.values(), start=1):
        point2d_index = len(observations[image_id])
        observations[image_id].append((float(pixel[0]), float(pixel[1]), point_id))
        points.append(
            SeedPoint(
                point_id=point_id,
                xyz=np.asarray(xyz, dtype=float),
                rgb=np.asarray(rgb, dtype=np.uint8),
                image_id=image_id,
                point2d_index=point2d_index,
            )
        )
    return points, observations


def _pack(file, values, format_string: str) -> None:
    if not isinstance(values, (tuple, list)):
        values = (values,)
    file.write(struct.pack("<" + format_string, *values))


def write_cameras_binary(path: Path, transforms: dict) -> None:
    with path.open("wb") as file:
        _pack(file, 1, "Q")
        _pack(
            file,
            (1, PINHOLE_MODEL_ID, int(transforms["w"]), int(transforms["h"])),
            "iiQQ",
        )
        _pack(
            file,
            tuple(float(transforms[key]) for key in ("fl_x", "fl_y", "cx", "cy")),
            "dddd",
        )


def write_images_binary(path: Path, images: list[ColmapImage]) -> None:
    with path.open("wb") as file:
        _pack(file, len(images), "Q")
        for image in images:
            _pack(file, image.image_id, "i")
            _pack(file, image.qvec.tolist(), "dddd")
            _pack(file, image.tvec.tolist(), "ddd")
            _pack(file, 1, "i")
            file.write(image.name.encode("utf-8") + b"\x00")
            _pack(file, len(image.observations), "Q")
            for u, v, point_id in image.observations:
                _pack(file, (u, v, point_id), "ddq")


def write_points3d_binary(path: Path, points: list[SeedPoint]) -> None:
    with path.open("wb") as file:
        _pack(file, len(points), "Q")
        for point in points:
            _pack(file, point.point_id, "Q")
            _pack(file, point.xyz.tolist(), "ddd")
            _pack(file, [int(value) for value in point.rgb], "BBB")
            _pack(file, 0.0, "d")
            _pack(file, 1, "Q")
            _pack(file, (point.image_id, point.point2d_index), "ii")


def write_text_model(
    output_dir: Path,
    transforms: dict,
    images: list[ColmapImage],
    points: list[SeedPoint],
) -> None:
    camera_line = " ".join(
        str(value)
        for value in (
            1,
            "PINHOLE",
            transforms["w"],
            transforms["h"],
            transforms["fl_x"],
            transforms["fl_y"],
            transforms["cx"],
            transforms["cy"],
        )
    )
    (output_dir / "cameras.txt").write_text(
        "# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n" + camera_line + "\n"
    )

    image_lines = ["# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME"]
    for image in images:
        image_lines.append(
            " ".join(
                str(value)
                for value in (
                    image.image_id,
                    *image.qvec,
                    *image.tvec,
                    1,
                    image.name,
                )
            )
        )
        image_lines.append(
            " ".join(
                f"{u} {v} {point_id}"
                for u, v, point_id in image.observations
            )
        )
    (output_dir / "images.txt").write_text("\n".join(image_lines) + "\n")

    point_lines = ["# POINT3D_ID X Y Z R G B ERROR TRACK[]"]
    for point in points:
        point_lines.append(
            " ".join(
                str(value)
                for value in (
                    point.point_id,
                    *point.xyz,
                    *point.rgb,
                    0.0,
                    point.image_id,
                    point.point2d_index,
                )
            )
        )
    (output_dir / "points3D.txt").write_text("\n".join(point_lines) + "\n")


def write_binary_ply(path: Path, points: list[SeedPoint]) -> None:
    vertices = np.empty(
        len(points),
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("nx", "<f4"),
            ("ny", "<f4"),
            ("nz", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    xyz = np.asarray([point.xyz for point in points], dtype=np.float32)
    rgb = np.asarray([point.rgb for point in points], dtype=np.uint8)
    vertices["x"], vertices["y"], vertices["z"] = xyz.T
    vertices["nx"] = vertices["ny"] = vertices["nz"] = 0.0
    vertices["red"], vertices["green"], vertices["blue"] = rgb.T
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(vertices)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property float nx\nproperty float ny\nproperty float nz\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n"
    )
    with path.open("wb") as file:
        file.write(header.encode("ascii"))
        vertices.tofile(file)


def prepare_images(
    dataset_dir: Path,
    source_dir: Path,
    names: list[str],
    resize: bool,
) -> None:
    images_dir = dataset_dir / "images"
    images_dir.mkdir(exist_ok=True)
    for name in names:
        source = source_dir / name
        destination = images_dir / name
        if source.resolve() != destination.resolve():
            shutil.copy2(source, destination)

    if not resize:
        return
    for divisor in (2, 4, 8):
        resized_dir = dataset_dir / f"images_{divisor}"
        resized_dir.mkdir(exist_ok=True)
        for name in names:
            with Image.open(source_dir / name) as image:
                size = (
                    max(1, image.width // divisor),
                    max(1, image.height // divisor),
                )
                image.resize(size, Image.Resampling.LANCZOS).save(resized_dir / name)


def encode_inverse_depth(depth_m: np.ndarray, scale: float) -> np.ndarray:
    """Encode inverse camera-Z depth for the Gaussian Splatting loader."""
    if scale <= 0.0:
        raise ValueError("inverse-depth scale must be positive")
    depth_m = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    inverse_depth = np.zeros(depth_m.shape, dtype=np.float32)
    inverse_depth[valid] = 1.0 / depth_m[valid]
    normalized = np.clip(inverse_depth / scale, 0.0, 1.0)
    return np.rint(normalized * 65535.0).astype(np.uint16)


def prepare_inverse_depths(
    dataset_dir: Path,
    frames: list[dict],
    names: list[str],
) -> float:
    """Write uint16 inverse-depth maps and return their common metric scale."""
    maximum_inverse_depth = 0.0
    depth_paths = []
    for frame, name in zip(frames, names):
        depth_path = depth_path_for_frame(dataset_dir, frame, name)
        depth_paths.append(depth_path)
        depth_m = np.load(depth_path, mmap_mode="r")
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        if np.any(valid):
            maximum_inverse_depth = max(
                maximum_inverse_depth,
                float(np.max(1.0 / depth_m[valid])),
            )
    if maximum_inverse_depth <= 0.0:
        raise ValueError("depth maps contain no valid pixels")

    # Keep the largest value below uint16 saturation despite float rounding.
    scale = maximum_inverse_depth * 1.001
    inverse_depth_dir = dataset_dir / "invdepth"
    inverse_depth_dir.mkdir(exist_ok=True)
    for depth_path, name in zip(depth_paths, names):
        encoded = encode_inverse_depth(np.load(depth_path), scale)
        Image.fromarray(encoded).save(inverse_depth_dir / name)
    return scale


def export_colmap_dataset(
    dataset_dir: Path,
    *,
    depth_stride: int = 10,
    view_stride: int = 1,
    voxel_size_m: float = 0.015,
    resize: bool = False,
) -> tuple[int, int]:
    dataset_dir = dataset_dir.resolve()
    transforms = json.loads((dataset_dir / "transforms.json").read_text())
    frames = transforms["frames"]
    if not frames:
        raise ValueError("transforms.json contains no frames")

    source_dir = dataset_dir / "input"
    if not source_dir.is_dir():
        source_dir = dataset_dir / "images"
    names = [Path(frame["file_path"]).name for frame in frames]
    missing = [name for name in names if not (source_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(f"missing {len(missing)} source images; first: {missing[0]}")

    sparse_dir = dataset_dir / "sparse" / "0"
    if sparse_dir.exists() and any(sparse_dir.iterdir()):
        raise FileExistsError(f"COLMAP model already exists: {sparse_dir}")
    sparse_dir.mkdir(parents=True, exist_ok=True)

    points, observations = build_seed_points(
        dataset_dir,
        transforms,
        source_dir,
        depth_stride=depth_stride,
        view_stride=view_stride,
        voxel_size_m=voxel_size_m,
    )
    images = []
    for frame_index, frame in enumerate(frames):
        rotation, translation = opengl_c2w_to_colmap(
            np.asarray(frame["transform_matrix"], dtype=float)
        )
        images.append(
            ColmapImage(
                image_id=frame_index + 1,
                name=names[frame_index],
                qvec=rotmat_to_qvec(rotation),
                tvec=translation,
                observations=tuple(observations[frame_index + 1]),
            )
        )

    write_cameras_binary(sparse_dir / "cameras.bin", transforms)
    write_images_binary(sparse_dir / "images.bin", images)
    write_points3d_binary(sparse_dir / "points3D.bin", points)
    write_binary_ply(sparse_dir / "points3D.ply", points)
    write_text_model(sparse_dir, transforms, images, points)
    inverse_depth_scale = prepare_inverse_depths(dataset_dir, frames, names)
    (sparse_dir / "depth_params.json").write_text(
        json.dumps(
            {
                Path(name).stem: {"scale": inverse_depth_scale, "offset": 0.0}
                for name in names
            },
            indent=2,
        )
        + "\n"
    )
    prepare_images(dataset_dir, source_dir, names, resize)
    return len(images), len(points)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export renderer transforms and RGB-D as a COLMAP/Gaussian-Splatting dataset."
    )
    parser.add_argument("--source-path", "-s", required=True, type=Path)
    parser.add_argument("--depth-stride", type=int, default=10)
    parser.add_argument("--view-stride", type=int, default=1)
    parser.add_argument("--voxel-size", type=float, default=0.015)
    parser.add_argument("--resize", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image_count, point_count = export_colmap_dataset(
        args.source_path,
        depth_stride=args.depth_stride,
        view_stride=args.view_stride,
        voxel_size_m=args.voxel_size,
        resize=args.resize,
    )
    print(
        f"COLMAP export complete: {image_count} cameras, {point_count} RGB-D seed points, "
        f"source={args.source_path.resolve()}"
    )


if __name__ == "__main__":
    main()
