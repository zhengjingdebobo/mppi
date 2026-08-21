# Basic configuration holder
from dataclasses import dataclass

@dataclass
class MPPIConfig:
    num_samples: int = 200
    horizon: int = 15
    lambda_: float = 1.0
