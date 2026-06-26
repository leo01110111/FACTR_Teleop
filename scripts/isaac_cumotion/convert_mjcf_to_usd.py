#!/usr/bin/env python3
"""Convert a MJCF XML file to USD using Isaac Sim 6's native MJCF importer."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mjcf", type=Path, help="Input MJCF XML file.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("generated/isaac_cumotion/usd"),
        help="Directory where Isaac writes the converted USD package.",
    )
    parser.add_argument("--headless", action="store_true", default=True)
    parser.add_argument("--no-headless", dest="headless", action="store_false")
    parser.add_argument("--fix-base", action="store_true")
    parser.add_argument("--merge-mesh", action="store_true")
    parser.add_argument("--debug-mode", action="store_true")
    parser.add_argument("--keep-mujoco-physics", action="store_true")
    parser.add_argument("--no-asset-transformer", action="store_true")
    args = parser.parse_args()

    mjcf_path = args.mjcf.resolve()
    output_dir = args.output_dir.resolve()
    if not mjcf_path.exists():
        raise FileNotFoundError(mjcf_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": bool(args.headless)})
    try:
        from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Usd

        enable_extension("isaacsim.asset.importer.mjcf")
        for _ in range(5):
            simulation_app.update()

        config = MJCFImporterConfig(
            mjcf_path=str(mjcf_path),
            usd_path=str(output_dir),
            import_scene=True,
            merge_mesh=bool(args.merge_mesh),
            fix_base=True if args.fix_base else None,
            run_asset_transformer=not bool(args.no_asset_transformer),
            run_multi_physics_conversion=not bool(args.keep_mujoco_physics),
            debug_mode=bool(args.debug_mode),
        )
        final_path = MJCFImporter(config).import_mjcf()
        stage = Usd.Stage.Open(final_path)
        prim_count = len(list(stage.Traverse())) if stage is not None else 0
        if prim_count == 0:
            raise RuntimeError(f"converted USD is empty: {final_path}")
        print(final_path)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
