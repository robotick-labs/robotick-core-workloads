# Copyright Robotick contributors
# SPDX-License-Identifier: Apache-2.0

"""
Wrapper script that reuses the canonical robotick-engine code policy checker.

The engine's `tools/code_policy_check.py` exposes a reusable
`run_policy_check` function. This wrapper locates that script (either via the
`--engine-root` flag, the `ROBOTICK_ENGINE_ROOT` environment variable, or by
looking for a sibling `robotick-engine` checkout) and forwards the check to it.
"""

import argparse
import importlib.util
import os
import sys
from pathlib import Path
from typing import Optional

CANONICAL_SCRIPT = Path("tools/code_policy_check.py")
EXCLUDED_DIRS = ["cpp/tests/external"]


def _resolve_engine_root(cli_root: Optional[str]) -> Path:
    if cli_root:
        return Path(cli_root).resolve()

    env_value = os.environ.get("ROBOTICK_ENGINE_ROOT")
    if env_value:
        return Path(env_value).resolve()

    # Assume the engine repo lives next to the core-workloads repo.
    return (Path(__file__).resolve().parents[1].parent / "robotick-engine").resolve()


def _load_canonical_module(script_path: Path):
    spec = importlib.util.spec_from_file_location("robotick_code_policy", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to import policy checker at {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "run_policy_check"):
        raise RuntimeError(
            f"{script_path} does not export run_policy_check (required for reuse)"
        )
    return module


def main():
    parser = argparse.ArgumentParser(
        description="Run the shared Robotick code policy checker"
    )
    parser.add_argument(
        "source_root",
        help="Root directory containing the sources to inspect",
    )
    parser.add_argument(
        "--engine-root",
        help=(
            "Path to the robotick-engine checkout containing tools/code_policy_check.py. "
            "Defaults to ROBOTICK_ENGINE_ROOT or a sibling ../robotick-engine directory."
        ),
    )
    args = parser.parse_args()

    engine_root = _resolve_engine_root(args.engine_root)
    script_path = engine_root / CANONICAL_SCRIPT
    if not script_path.is_file():
        raise SystemExit(
            f"Unable to locate canonical code policy script at {script_path}. "
            "Provide --engine-root or set ROBOTICK_ENGINE_ROOT."
        )

    module = _load_canonical_module(script_path)
    module.run_policy_check(
        args.source_root, header_mode="exact", exclude_dirs=EXCLUDED_DIRS
    )


if __name__ == "__main__":
    main()
