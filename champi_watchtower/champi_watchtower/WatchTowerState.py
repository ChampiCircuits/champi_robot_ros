#!/usr/bin/env python3

from enum import Enum

class WatchtowerState(Enum):
    """State machine states for watchtower node."""
    INIT = "Initialization"
    CALIBRATION = "Calibration"
    RUNNING = "Running"