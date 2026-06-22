"""Inverse and forward kinematics for a 3-DOF hexapod leg (coxa-femur-tibia)."""

import numpy as np
from dataclasses import dataclass


@dataclass
class LegConfig:
    coxa_length: float
    femur_length: float
    tibia_length: float
    mount_x: float
    mount_y: float
    mount_angle: float  # radians, leg direction from body center


class LegIK:
    """3-DOF leg inverse/forward kinematics.

    Convention (body frame): X forward, Y left, Z up.
    Joint angles in radians:
      - coxa:  rotation about Z (leg sweep)
      - femur: rotation about local Y (leg elevation)
      - tibia: rotation about local Y (knee bend)
    """

    def __init__(self, config: LegConfig):
        self.cfg = config

    def inverse(self, x: float, y: float, z: float) -> tuple:
        """Compute (coxa, femur, tibia) angles for foot at (x, y, z) in body frame.

        z is relative to the coxa joint height (negative = below body).
        """
        c = self.cfg
        cos_a = np.cos(c.mount_angle)
        sin_a = np.sin(c.mount_angle)

        dx = x - c.mount_x
        dy = y - c.mount_y

        lx = dx * cos_a + dy * sin_a
        ly = -dx * sin_a + dy * cos_a

        coxa = np.arctan2(ly, lx)

        r_horizontal = np.sqrt(lx ** 2 + ly ** 2) - c.coxa_length
        d = np.sqrt(r_horizontal ** 2 + z ** 2)

        if d > (c.femur_length + c.tibia_length):
            d = c.femur_length + c.tibia_length - 1e-6
        if d < abs(c.femur_length - c.tibia_length):
            d = abs(c.femur_length - c.tibia_length) + 1e-6

        cos_knee = (c.femur_length ** 2 + c.tibia_length ** 2 - d ** 2) / (
            2 * c.femur_length * c.tibia_length
        )
        cos_knee = np.clip(cos_knee, -1.0, 1.0)
        tibia = -(np.pi - np.arccos(cos_knee))

        cos_beta = (c.femur_length ** 2 + d ** 2 - c.tibia_length ** 2) / (
            2 * c.femur_length * d
        )
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)

        femur = np.arctan2(-z, r_horizontal) - beta

        return float(coxa), float(femur), float(tibia)

    def forward(self, coxa: float, femur: float, tibia: float) -> tuple:
        """Compute foot position (x, y, z) in body frame from joint angles."""
        c = self.cfg

        foot_r = (
            c.coxa_length
            + c.femur_length * np.cos(femur)
            + c.tibia_length * np.cos(femur + tibia)
        )
        foot_z = -(
            c.femur_length * np.sin(femur)
            + c.tibia_length * np.sin(femur + tibia)
        )

        lx = foot_r * np.cos(coxa)
        ly = foot_r * np.sin(coxa)

        cos_a = np.cos(c.mount_angle)
        sin_a = np.sin(c.mount_angle)

        x = lx * cos_a - ly * sin_a + c.mount_x
        y = lx * sin_a + ly * cos_a + c.mount_y

        return float(x), float(y), float(foot_z)


class HexapodModel:
    """Complete hexapod with 6 legs."""

    LEG_NAMES = [
        "right_front",
        "right_middle",
        "right_rear",
        "left_rear",
        "left_middle",
        "left_front",
    ]

    LEG_MOUNT_ANGLES_DEG = [-30.0, -90.0, -150.0, 150.0, 90.0, 30.0]

    def __init__(
        self,
        body_radius: float = 0.07,
        coxa_length: float = 0.09,
        femur_length: float = 0.185,
        tibia_length: float = 0.250,
        body_height: float = 0.15,
    ):
        self.body_radius = body_radius
        self.body_height = body_height
        self.num_legs = 6
        self.legs: list[LegIK] = []
        self.rest_positions: list[tuple] = []

        for i in range(self.num_legs):
            angle_rad = np.radians(self.LEG_MOUNT_ANGLES_DEG[i])
            mount_x = body_radius * np.cos(angle_rad)
            mount_y = body_radius * np.sin(angle_rad)

            config = LegConfig(
                coxa_length=coxa_length,
                femur_length=femur_length,
                tibia_length=tibia_length,
                mount_x=mount_x,
                mount_y=mount_y,
                mount_angle=angle_rad,
            )
            leg = LegIK(config)
            self.legs.append(leg)

            reach = coxa_length + femur_length * 0.7
            rest_x = mount_x + reach * np.cos(angle_rad)
            rest_y = mount_y + reach * np.sin(angle_rad)
            rest_z = -body_height
            self.rest_positions.append((rest_x, rest_y, rest_z))

    def compute_ik_all(self, foot_positions: list) -> list:
        """Compute joint angles for all 6 legs given foot positions in body frame."""
        all_angles = []
        for i in range(self.num_legs):
            x, y, z = foot_positions[i]
            angles = self.legs[i].inverse(x, y, z)
            all_angles.append(angles)
        return all_angles

    def get_rest_angles(self) -> list:
        """Compute joint angles for the default standing pose."""
        return self.compute_ik_all(self.rest_positions)
