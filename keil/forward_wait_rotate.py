import argparse
import importlib.util
import sys
import time
from pathlib import Path


def load_car_controller(source_file: Path):
    spec = importlib.util.spec_from_file_location("car_control_source", source_file)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load {source_file}")

    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.CarController


def main():
    parser = argparse.ArgumentParser(description="Move forward 1m, wait 15s, then rotate 90 degrees.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port, for example /dev/ttyUSB0 or COM3.")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--distance", type=float, default=1.0, help="Forward distance in meters.")
    parser.add_argument("--wait", type=float, default=15.0, help="Wait time in seconds after moving.")
    parser.add_argument("--angle", type=float, default=90.0, help="Rotate angle in degrees. Positive is CCW.")
    parser.add_argument("--move-timeout", type=float, default=30.0)
    parser.add_argument("--rotate-timeout", type=float, default=30.0)
    args = parser.parse_args()

    source_file = Path(__file__).with_name("car_control.py")
    CarController = load_car_controller(source_file)

    car = CarController(port=args.port, baudrate=args.baudrate)
    if not car.connect():
        raise RuntimeError(f"Failed to connect to car on {args.port}")

    try:
        car.initialize_car()

        print(f"Move forward {args.distance} m")
        if not car.move(args.distance, timeout=args.move_timeout):
            raise RuntimeError("Move command failed or timed out")

        print(f"Wait {args.wait} s")
        time.sleep(args.wait)

        print(f"Rotate {args.angle} deg")
        if not car.rotate(args.angle, timeout=args.rotate_timeout):
            raise RuntimeError("Rotate command failed or timed out")

        print("Done")
    finally:
        car.stop()
        time.sleep(0.1)
        car.disconnect()


if __name__ == "__main__":
    main()
