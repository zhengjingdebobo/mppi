# EXACT MPPI Core

JAX-based MPPI controller package used by the examples in `EXACT_MPPI_core/example/`.

This package now contains only the maintained JAX path.

## Structure

```text
exact_mppi/
├── config/      # Config helpers
├── mppi_jax/    # JAX controller, optimizer, critics, models, tools
├── path/        # Path search and reference path utilities
└── utils/       # Grid conversion and geometry helpers

example/
├── corridor_dynamic_random_jax/
├── corridor_jax/
├── corridor_jax_cluttered/
├── narrow_gaps_jax/
└── T-shape_trap_jax/
```

## Installation

From the repository root:

```bash
source .exact_mppi/bin/activate
python -m pip install -e ./ir-sim_mppi
python -m pip install -e ./EXACT_MPPI_core
```

GPU is the recommended setup for the JAX examples. If you have an NVIDIA GPU, install the matching JAX CUDA wheel before installing the core package:

```bash
python -m pip install -U "jax[cuda12]"
python -m pip install -e ./EXACT_MPPI_core
```

For CPU-only execution:

```bash
python -m pip install -U "jax[cpu]"
python -m pip install -e ./EXACT_MPPI_core
```

If plain `pip install -e ./EXACT_MPPI_core` is run outside the intended environment, activate `.exact_mppi` first.

## Quick Check

```bash
python -c "import irsim; from exact_mppi.mppi_jax.controller import MPPIController; print('setup ok')"
```

## Run Examples

```bash
python EXACT_MPPI_core/example/corridor_dynamic_random_jax/mppi_jax_anyshape.py --robot-shape f
python EXACT_MPPI_core/example/corridor_jax/mppi_jax_test.py
python EXACT_MPPI_core/example/narrow_gaps_jax/mppi_jax_test.py
```

Each example reports the active JAX backend and device list before starting, so you can confirm whether it is running on CPU or GPU.

## Minimal Usage

```python
import yaml
from exact_mppi.mppi_jax.controller import MPPIController

with open("example/corridor_jax/planner.yaml", "r", encoding="utf-8") as f:
    planner_cfg = yaml.safe_load(f)

mppi = MPPIController(**planner_cfg["MPPI"])
mppi.setRectangleFootprint(planner_cfg["MPPI"]["vertices"])
```

## Notes

The maintained public surface is centered on `exact_mppi.mppi_jax`, `exact_mppi.path`, and `exact_mppi.utils`.