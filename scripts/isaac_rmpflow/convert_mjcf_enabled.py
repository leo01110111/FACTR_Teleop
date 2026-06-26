#!/usr/bin/env python3
"""Convert MJCF to USD with Isaac's MJCF importer extension enabled.

IsaacLab's stock `scripts/tools/convert_mjcf.py` currently assumes the MJCF
importer command is registered by the selected experience. In our headless
bring-up, it was not, so this wrapper enables `isaacsim.asset.importer.mjcf`
before constructing the same IsaacLab converter.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("input", type=Path, help="Input MJCF XML file.")
parser.add_argument("output", type=Path, help="Output USD file.")
parser.add_argument("--fix-base", action="store_true", default=False)
parser.add_argument("--import-sites", action="store_true", default=False)
parser.add_argument("--make-instanceable", action="store_true", default=False)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaacsim.core.utils.extensions import enable_extension

from pxr import Usd

from isaaclab.sim.converters import MjcfConverter, MjcfConverterCfg
from isaaclab.utils.assets import check_file_path
from isaaclab.utils.dict import print_dict


def main() -> None:
    mjcf_path = args_cli.input.resolve()
    usd_path = args_cli.output.resolve()
    if not check_file_path(str(mjcf_path)):
        raise ValueError(f"Invalid MJCF path: {mjcf_path}")

    enable_extension("isaacsim.asset.importer.mjcf")
    for _ in range(5):
        simulation_app.update()

    cfg = MjcfConverterCfg(
        asset_path=str(mjcf_path),
        usd_dir=str(usd_path.parent),
        usd_file_name=usd_path.name,
        fix_base=args_cli.fix_base,
        import_sites=args_cli.import_sites,
        force_usd_conversion=True,
        make_instanceable=args_cli.make_instanceable,
    )

    print("-" * 80)
    print(f"Input MJCF file: {mjcf_path}")
    print("MJCF importer config:")
    print_dict(cfg.to_dict(), nesting=0)
    print("-" * 80)

    converter = MjcfConverter(cfg)
    if not os.path.exists(converter.usd_path):
        raise RuntimeError(f"MJCF converter did not produce USD: {converter.usd_path}")
    stage = Usd.Stage.Open(converter.usd_path)
    prim_count = len(list(stage.Traverse())) if stage is not None else 0
    if prim_count == 0:
        raise RuntimeError(f"MJCF converter produced an empty USD: {converter.usd_path}")
    print(f"Generated USD file: {converter.usd_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
