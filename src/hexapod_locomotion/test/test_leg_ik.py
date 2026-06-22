"""Unit tests for hexapod leg inverse kinematics."""

import math
import pytest
from hexapod_locomotion.leg_ik import LegIK, LegConfig, HexapodModel


def _make_leg(mount_angle=0.0):
    return LegIK(LegConfig(
        coxa_length=0.09,
        femur_length=0.185,
        tibia_length=0.250,
        mount_x=0.07 * math.cos(mount_angle),
        mount_y=0.07 * math.sin(mount_angle),
        mount_angle=mount_angle,
    ))


class TestLegIK:
    """Verify forward/inverse consistency for a single leg."""

    def test_forward_inverse_roundtrip(self):
        leg = _make_leg(0.0)
        coxa_in, femur_in, tibia_in = 0.1, -0.3, -0.5
        x, y, z = leg.forward(coxa_in, femur_in, tibia_in)
        coxa_out, femur_out, tibia_out = leg.inverse(x, y, z)

        assert abs(coxa_in - coxa_out) < 1e-6
        assert abs(femur_in - femur_out) < 1e-6
        assert abs(tibia_in - tibia_out) < 1e-6

    def test_rest_position_reachable(self):
        leg = _make_leg(-math.pi / 6)
        mount_angle = -math.pi / 6
        reach = 0.09 + 0.185 * 0.7
        rest_x = 0.07 * math.cos(mount_angle) + reach * math.cos(mount_angle)
        rest_y = 0.07 * math.sin(mount_angle) + reach * math.sin(mount_angle)
        rest_z = -0.15

        coxa, femur, tibia = leg.inverse(rest_x, rest_y, rest_z)

        assert -math.pi < coxa < math.pi
        assert -math.pi < femur < math.pi
        assert -math.pi < tibia < 0

    def test_forward_inverse_multiple_angles(self):
        leg = _make_leg(math.pi / 2)
        for coxa_in in [-0.3, 0.0, 0.3]:
            for femur_in in [-0.5, -0.2, 0.1]:
                tibia_in = -0.8
                x, y, z = leg.forward(coxa_in, femur_in, tibia_in)
                coxa_out, femur_out, tibia_out = leg.inverse(x, y, z)

                assert abs(coxa_in - coxa_out) < 1e-5, f"coxa mismatch at ({coxa_in}, {femur_in})"
                assert abs(femur_in - femur_out) < 1e-5, f"femur mismatch at ({coxa_in}, {femur_in})"
                assert abs(tibia_in - tibia_out) < 1e-5, f"tibia mismatch at ({coxa_in}, {femur_in})"

    def test_unreachable_clamped(self):
        leg = _make_leg(0.0)
        coxa, femur, tibia = leg.inverse(10.0, 0.0, 0.0)
        assert math.isfinite(coxa)
        assert math.isfinite(femur)
        assert math.isfinite(tibia)


class TestHexapodModel:
    """Verify the full 6-leg model."""

    def test_rest_angles_computed(self):
        model = HexapodModel()
        angles = model.get_rest_angles()
        assert len(angles) == 6
        for coxa, femur, tibia in angles:
            assert math.isfinite(coxa)
            assert math.isfinite(femur)
            assert math.isfinite(tibia)

    def test_all_rest_positions_reachable(self):
        model = HexapodModel()
        angles = model.compute_ik_all(model.rest_positions)
        for i, (coxa, femur, tibia) in enumerate(angles):
            x, y, z = model.legs[i].forward(coxa, femur, tibia)
            rx, ry, rz = model.rest_positions[i]
            assert abs(x - rx) < 1e-5, f"leg {i} x mismatch"
            assert abs(y - ry) < 1e-5, f"leg {i} y mismatch"
            assert abs(z - rz) < 1e-5, f"leg {i} z mismatch"

    def test_six_legs_created(self):
        model = HexapodModel()
        assert model.num_legs == 6
        assert len(model.legs) == 6
        assert len(model.rest_positions) == 6
