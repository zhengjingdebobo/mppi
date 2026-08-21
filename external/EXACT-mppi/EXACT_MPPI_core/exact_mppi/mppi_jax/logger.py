# import logging

# logger = logging.getLogger("[MPPIXXXXX]")

import logging

class ColorFormatter(logging.Formatter):
    COLORS = {
        logging.DEBUG: "\033[36m",     # Cyan
        logging.INFO: "\033[32m",      # Green
        logging.WARNING: "\033[33m",   # Yellow
        logging.ERROR: "\033[31m",     # Red
        logging.CRITICAL: "\033[1;31m" # Bold Red
    }
    RESET = "\033[0m"
    
    BLUE = "\033[34m"
    PURPLE = "\033[35m"

    def format(self, record):
        color = self.COLORS.get(record.levelno, "")
        record.name = f"{self.PURPLE}{record.name}{self.RESET}"
        record.levelname = f"{color}{record.levelname}{self.RESET}"
        record.msg = f"{color}{record.msg}{self.RESET}"
        return super().format(record)
    
logger = logging.getLogger("MPPI")
logger.setLevel(logging.DEBUG)

if not logger.handlers:
    handler = logging.StreamHandler()
    formatter = ColorFormatter(
        "[%(name)s] %(levelname)s: %(message)s"
    )
    handler.setFormatter(formatter)
    logger.addHandler(handler)

logger.propagate = False