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
    TAKE_2_BOXES                   = 6
    BRING_2_BOXES_ON_TOP           = 7
    PUT_2_LAST_BOXES_ON_THE_GROUND = 8
    PREPARE_TOP_PUSHER             = 9
    GRAB_AND_SORT_2_BOXES_FROM_LIFT = 10
    PUSH_2_BOXES_OUT               = 11
    OPEN_EXIT_RAMP                 = 12

    # OTHER COMMON ACTIONS
    MOVE                           = 100
    DETECT_PLATFORM                = 101
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
