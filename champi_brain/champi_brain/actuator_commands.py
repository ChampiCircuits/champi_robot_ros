#!/usr/bin/env python3
"""
ActuatorCommand enum

Must stay in sync with ActuatorCommand in:
  champi_hw_interface/include/champi_hw_interface/hw_actuators.h
Values are the integer positions of each entry in that C++ enum.
"""

from enum import IntEnum
from typing import Optional


class ActuatorCommand(IntEnum):
    RESET_ACTUATORS                = 0
    STOP_ALL_MOTORS                = 1
    ENABLE_ALL_MOTORS              = 2
    GET_READY                      = 3
    THERMOMETER_LOWER_SERVO        = 4
    THERMOMETER_RAISE_SERVO        = 5

    LOWER_LEFT_ARM                 = 6
    GET_READY_LEFT_ARM             = 7
    LET_GO_ELEMENTS_LEFT_ARM       = 8
    
    LOWER_RIGHT_ARM                = 9
    GET_READY_RIGHT_ARM            = 10
    LET_GO_ELEMENTS_RIGHT_ARM      = 11

    STORE_PENDING_MASK             = 12
    PUMPS_ON = 13
    PUMPS_OFF = 14

    # OTHER COMMON ACTIONS (not sent to STM)
    MOVE                           = 100
    DETECT_NUTBOXES                = 101
    WAIT                           = 102
    ADD_POINTS                     = 103

    def __str__(self) -> str:
        return self._name_


def actuator_name_to_id(action_name: str) -> Optional[int]:
    """Convert an actuator command name to its integer ID.
    Returns None if the name is not a valid ActuatorCommand.
    """
    try:
        return ActuatorCommand[action_name].value
    except KeyError:
        return None
