from enum import Enum


class FlightMode(Enum):
    """
    Available flight modes:
        PRE_TAKEOFF (Before taking off, check that noone is nearby)
        MISSION ("Normal mode", fly with predefined trajectory)
        PRE_LANDING (Go to safe position before full landing)
        LANDING (self-explanatory)
        EMERGENCY_RETREAT (retreat to safe position when landing)
    """

    PRE_TAKEOFF = 0
    MISSION = 1
    PRE_LANDING = 2
    LANDING = 3
    EMERGENCY_RETREAT = 4
