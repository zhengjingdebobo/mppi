"""exact_mppi_jax package.

Bootstrap the active exact_mppi Python environment for ROS entrypoints.

The ament/setuptools console scripts in this workspace may be generated with a
system Python shebang even when the workspace is built from an activated
virtualenv. When that happens, sourcing ``.exact_mppi`` before ``ros2 launch``
sets ``VIRTUAL_ENV`` but does not automatically expose that environment's
site-packages to the system interpreter running the ROS node. Add the active
virtualenv's site-packages explicitly so the bridge can import ``exact_mppi``
and its runtime dependencies.
"""

from __future__ import annotations

import glob
import os
from pathlib import Path
import sys


def _prepend_if_exists(path: str) -> None:
	if os.path.isdir(path):
		if path in sys.path:
			sys.path.remove(path)
		sys.path.insert(0, path)


def _bootstrap_runtime_pythonpath() -> None:
	candidates: list[str] = []

	venv = os.environ.get("VIRTUAL_ENV")
	if venv:
		candidates.extend(glob.glob(os.path.join(venv, "lib", "python*", "site-packages")))

	current = Path(__file__).resolve()
	for parent in current.parents:
		repo_venv = parent / ".exact_mppi" / "lib"
		if repo_venv.is_dir():
			candidates.extend(glob.glob(str(repo_venv / "python*" / "site-packages")))

		repo_core = parent / "EXACT_MPPI_core"
		if repo_core.is_dir():
			sys.path.insert(0, str(repo_core))
			break

	for candidate in reversed(candidates):
		_prepend_if_exists(candidate)


_bootstrap_runtime_pythonpath()
