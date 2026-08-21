import os
import time


def time_it(fn):
    def wrapper(*args, **kwargs):
        start = time.time()
        result = fn(*args, **kwargs)
        end = time.time()
        print(f"{fn.__name__} took {end - start:.4f}s")
        return result
    return wrapper


def file_check(path: str) -> str:
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return path


def repeat_mk_dirs(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path
