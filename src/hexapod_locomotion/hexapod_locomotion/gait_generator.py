"""Gait pattern generator for hexapod walking."""

import numpy as np
from enum import Enum


class GaitType(Enum):
    TRIPOD = "tripod"
    WAVE = "wave"
    RIPPLE = "ripple"


PHASE_OFFSETS = {
    GaitType.TRIPOD: [0.0, 0.5, 0.0, 0.5, 0.0, 0.5],
    GaitType.WAVE: [0.0, 2 / 6, 4 / 6, 1 / 6, 3 / 6, 5 / 6],
    GaitType.RIPPLE: [0.0, 1 / 3, 2 / 3, 1 / 6, 1 / 2, 5 / 6],
}

DUTY_FACTORS = {
    GaitType.TRIPOD: 0.5,
    GaitType.WAVE: 5 / 6,
    GaitType.RIPPLE: 2 / 3,
}


class GaitGenerator:
    """Generates foot trajectories for different gait patterns.

    The gait cycle is normalized to [0, 1). Each leg has a phase offset
    that determines when it lifts (transfer) vs pushes (support).
    """

    def __init__(self):
        self.gait_type = GaitType.TRIPOD
        self.step_height = 0.04   # meters, foot lift height
        self.cycle_period = 1.0   # seconds per full gait cycle
        self._phase = 0.0

    def set_gait(self, gait_type: GaitType):
        self.gait_type = gait_type
        self._phase = 0.0

    def update(self, dt: float, vx: float, vy: float, omega: float,
               rest_positions: list) -> list:
        """Advance gait by dt seconds and return foot positions for all 6 legs.

        Args:
            dt: time step in seconds
            vx: forward velocity command (m/s)
            vy: lateral velocity command (m/s)
            omega: yaw rate command (rad/s)
            rest_positions: list of 6 (x, y, z) rest foot positions in body frame

        Returns:
            list of 6 (x, y, z) foot target positions
        """
        speed = np.sqrt(vx ** 2 + vy ** 2 + omega ** 2)

        if speed < 1e-4:
            return list(rest_positions)

        self._phase = (self._phase + dt / self.cycle_period) % 1.0

        offsets = PHASE_OFFSETS[self.gait_type]
        duty = DUTY_FACTORS[self.gait_type]

        positions = []
        for i in range(6):
            leg_phase = (self._phase + offsets[i]) % 1.0
            rx, ry, rz = rest_positions[i]

            angle = np.arctan2(ry, rx)
            r = np.sqrt(rx ** 2 + ry ** 2)

            stride_x = vx - omega * r * np.sin(angle)
            stride_y = vy + omega * r * np.cos(angle)

            stride_scale = self.cycle_period * 0.5

            if leg_phase < duty:
                t = leg_phase / duty
                x = rx - stride_x * stride_scale * (t - 0.5)
                y = ry - stride_y * stride_scale * (t - 0.5)
                z = rz
            else:
                t = (leg_phase - duty) / (1.0 - duty)
                x = rx + stride_x * stride_scale * (0.5 - t)
                y = ry + stride_y * stride_scale * (0.5 - t)
                z = rz + self.step_height * np.sin(np.pi * t)

            positions.append((x, y, z))

        return positions

    def reset(self):
        self._phase = 0.0
