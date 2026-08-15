from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path

import imageio.v2 as imageio

from quick_support_demo.chrono_demo.build_scm_pit import build_scm_pit
from quick_support_demo.chrono_demo.build_support_proxy import build_support_proxy
from quick_support_demo.chrono_demo.build_world import add_box, add_perimeter_floor, build_system, make_contact_material
from quick_support_demo.chrono_demo.chrono_import import import_chrono
from quick_support_demo.config import PROJECT_ROOT, load_demo_config


chrono, veh = import_chrono()
import pychrono.irrlicht as irr


def apply_smoke_overrides(cfg: dict) -> None:
    cfg["world"]["world"]["timestep_s"] = 0.001
    cfg["world"]["world"]["settle_time_s"] = 1.5
    cfg["terrain"]["pit"]["grid_spacing_m"] = 0.035


def color_body(body: object, color: tuple[float, float, float]) -> None:
    model = body.GetVisualModel()
    for idx in range(model.GetNumShapes()):
        model.GetShape(idx).SetColor(chrono.ChColor(*color))


def add_marker(system: object, name: str, xy: tuple[float, float], radius: float, color: tuple[float, float, float]) -> object:
    marker = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, radius, 0.012, 1000.0, True, False)
    marker.SetName(name)
    marker.SetFixed(True)
    marker.SetPos(chrono.ChVector3d(float(xy[0]), float(xy[1]), 0.006))
    marker.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(*color))
    system.Add(marker)
    return marker


def add_scene_extras(system: object, candidates: dict) -> None:
    material = make_contact_material(0.5)
    add_marker(system, "sand_candidate_marker", tuple(candidates["sand"]["base_xy_m"]), 0.10, (0.10, 0.45, 0.95))
    add_marker(system, "rigid_candidate_marker", tuple(candidates["rigid"]["base_xy_m"]), 0.10, (0.05, 0.70, 0.25))
    add_box(
        system,
        "inspection_target",
        (0.55, 0.035, 0.35),
        (0.0, 1.35, 0.25),
        material,
        600.0,
        fixed=True,
        color=(0.95, 0.95, 0.10),
        collide=False,
    )
    add_box(
        system,
        "rear_scale_wall",
        (2.6, 0.04, 0.55),
        (0.0, 1.48, 0.235),
        material,
        800.0,
        fixed=True,
        color=(0.22, 0.24, 0.26),
        collide=False,
    )


def setup_visualizer(system: object, width: int, height: int) -> object:
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle("Chronos quick support demo")
    vis.SetWindowSize(width, height)
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.8, -2.4, 1.25), chrono.ChVector3d(0.0, 0.0, 0.05))
    vis.AddTypicalLights()
    return vis


def render_frame(vis: object, frame_path: Path | None = None) -> None:
    vis.BeginScene()
    vis.Render()
    if frame_path is not None:
        vis.WriteImageToFile(str(frame_path))
    vis.EndScene()


def encode_video(frame_dir: Path, output_path: Path, fps: int) -> None:
    frames = sorted(frame_dir.glob("frame_*.bmp"))
    if not frames:
        raise RuntimeError(f"No captured frames found in {frame_dir}")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with imageio.get_writer(output_path, fps=fps, codec="libx264", quality=8) as writer:
        for frame in frames:
            writer.append_data(imageio.imread(frame))


def run_visualization(args: argparse.Namespace) -> Path | None:
    cfg = load_demo_config()
    if args.smoke:
        apply_smoke_overrides(cfg)

    robot_cfg = cfg["robots"][args.robot]
    candidates = cfg["candidates"]["candidates"]
    system = build_system(cfg["world"])
    add_perimeter_floor(system, cfg["world"], cfg["terrain"])
    terrain = build_scm_pit(system, cfg["terrain"], visualization_mesh=True)
    terrain.SetColor(chrono.ChColor(0.74, 0.58, 0.34))
    terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.08)
    add_scene_extras(system, candidates)

    sand_xy = tuple(candidates["sand"]["base_xy_m"])
    start_xy = (sand_xy[0], -1.10)
    z0 = float(robot_cfg["robot"]["body_com_height_m"]) + 0.02
    body = build_support_proxy(system, robot_cfg, start_xy)
    body.SetFixed(True)
    body.EnableCollision(False)
    color_body(body, (0.05, 0.28, 0.92) if args.robot == "go1" else (0.95, 0.62, 0.05))

    vis = setup_visualizer(system, args.width, args.height)

    frame_root = PROJECT_ROOT / "quick_support_demo" / "outputs" / "frames" / datetime.now().strftime("%Y%m%d_%H%M%S")
    frame_root.mkdir(parents=True, exist_ok=True)
    video_path = PROJECT_ROOT / "quick_support_demo" / "outputs" / "videos" / f"{frame_root.name}_{args.robot}_irrlicht.mp4"

    dt = float(cfg["world"]["world"]["timestep_s"])
    total_time = float(args.duration)
    approach_time = min(2.5, total_time * 0.45)
    settle_start_done = False
    next_capture = 0.0
    frame_idx = 0
    sim_time = 0.0

    while vis.Run() and sim_time < total_time:
        if sim_time < approach_time:
            alpha = sim_time / max(approach_time, 1.0e-6)
            ease = 3.0 * alpha * alpha - 2.0 * alpha * alpha * alpha
            x = (1.0 - ease) * start_xy[0] + ease * sand_xy[0]
            y = (1.0 - ease) * start_xy[1] + ease * sand_xy[1]
            z = z0 + 0.025 * (1.0 - ease)
            body.SetPos(chrono.ChVector3d(x, y, z))
            body.SetRot(chrono.QUNIT)
            system.DoStepDynamics(dt)
        else:
            if not settle_start_done:
                body.SetFixed(False)
                body.EnableCollision(True)
                body.SetLinVel(chrono.ChVector3d(0.0, 0.0, 0.0))
                body.SetAngVelParent(chrono.ChVector3d(0.0, 0.0, 0.0))
                body.SetPos(chrono.ChVector3d(sand_xy[0], sand_xy[1], z0))
                settle_start_done = True
            terrain.Synchronize(system.GetChTime())
            system.DoStepDynamics(dt)
            terrain.Advance(dt)

        if args.record and sim_time >= next_capture:
            render_frame(vis, frame_root / f"frame_{frame_idx:05d}.bmp")
            next_capture += 1.0 / float(args.fps)
            frame_idx += 1
        elif not args.record:
            render_frame(vis)

        sim_time += dt

    if args.record:
        encode_video(frame_root, video_path, args.fps)
        if args.keep_frames:
            print(frame_root)
        else:
            shutil.rmtree(frame_root)
        return video_path
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render the quick support demo with Chrono Irrlicht.")
    parser.add_argument("--robot", choices=["go1", "spot"], default="go1")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--smoke", action="store_true", help="Use coarser SCM settings for faster capture.")
    parser.add_argument("--record", action="store_true", help="Capture frames and encode an MP4.")
    parser.add_argument("--xvfb", action="store_true", help="Run Irrlicht on a virtual X display.")
    parser.add_argument("--keep-frames", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.xvfb:
        from xvfbwrapper import Xvfb

        with Xvfb(width=args.width, height=args.height, colordepth=24):
            video_path = run_visualization(args)
    else:
        video_path = run_visualization(args)
    if video_path is not None:
        print(video_path)


if __name__ == "__main__":
    main()
