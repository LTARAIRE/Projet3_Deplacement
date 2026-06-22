"""Tests unitaires de la cinématique et de la démarche PhantomX."""

import math
import numpy as np
import pytest

from hexapod_locomotion import phantom_kinematics as pk


# ─────────────────────────────── FK / IK ────────────────────────────────────
def test_ik_fk_roundtrip():
    """L'IK doit retrouver une configuration plaçant le pied sur la cible."""
    rng = np.random.default_rng(0)
    max_err = 0.0
    for _ in range(200):
        leg = int(rng.integers(0, 6))
        q = [rng.uniform(-0.6, 0.6), rng.uniform(-1.3, -0.3), rng.uniform(-1.6, -0.5)]
        target = pk.fk(leg, q)
        q_sol = pk.ik(leg, target, q_seed=pk.REST_Q)
        reached = pk.fk(leg, q_sol)
        max_err = max(max_err, np.linalg.norm(reached - target))
    assert max_err < 1e-3, f"erreur IK trop grande: {max_err*1000:.2f} mm"


def test_joint_limits_respected():
    """Toute solution IST doit rester dans les limites des joints."""
    for leg in range(6):
        q = pk.ik(leg, pk.REST_FEET[leg])
        assert np.all(np.abs(q) <= pk.JOINT_LIMIT + 1e-9)


# ───────────────────────── Pose de repos symétrique ─────────────────────────
def test_rest_pose_level():
    """Tous les pieds au repos sont à la même hauteur (pose de niveau)."""
    zs = [f[2] for f in pk.REST_FEET]
    assert max(zs) - min(zs) < 1e-6


def test_rest_pose_left_right_symmetry():
    """Symétrie gauche/droite : pieds miroir en Y, même X et Z."""
    # indices : 0 rf,1 rm,2 rr,3 lf,4 lm,5 lr  → paires (rf,lf),(rm,lm),(rr,lr)
    # tolérance 1 mm : les yaw de montage de l'URDF sont légèrement tronqués
    # (0.7854 ≈ π/4) d'où une asymétrie résiduelle ~0.03 mm.
    for right, left in [(0, 3), (1, 4), (2, 5)]:
        fr, fl = pk.REST_FEET[right], pk.REST_FEET[left]
        assert fr[0] == pytest.approx(fl[0], abs=1e-3)   # même X
        assert fr[1] == pytest.approx(-fl[1], abs=1e-3)  # Y opposé
        assert fr[2] == pytest.approx(fl[2], abs=1e-3)   # même Z


def test_rest_feet_below_body():
    """Les pieds sont sous le corps (z < 0 dans base_link)."""
    assert all(f[2] < 0 for f in pk.REST_FEET)


# ─────────────────────── Sens de marche (le coeur du bug) ───────────────────
def test_foot_velocity_opposes_forward_command():
    """Marche avant (vx>0) : le pied au sol pousse vers l'arrière (vfx<0)."""
    vfx, vfy = pk.foot_ground_velocity(0.2, -0.1, vx=0.1, vy=0.0, omega=0.0)
    assert vfx < 0
    assert vfy == pytest.approx(0.0, abs=1e-9)


def test_foot_velocity_opposes_lateral_command():
    """Strafe gauche (vy>0) : le pied pousse vers la droite (vfy<0)."""
    vfx, vfy = pk.foot_ground_velocity(0.2, -0.1, vx=0.0, vy=0.1, omega=0.0)
    assert vfy < 0


def test_rotation_gives_tangential_foot_velocity():
    """Rotation pure (omega>0, CCW) : la vitesse du pied est tangentielle."""
    rx, ry = 0.2, 0.0   # pied droit devant, sur +X
    vfx, vfy = pk.foot_ground_velocity(rx, ry, vx=0.0, vy=0.0, omega=0.5)
    # v_pied = -(omega × r) ; pour r=+X, omega+Z → omega×r = +Y, donc vfy<0
    assert abs(vfx) < 1e-9
    assert vfy < 0


def test_stance_foot_moves_backward_for_forward_walk():
    """Sur la phase d'appui, marche avant ⇒ le pied recule (dx décroissant)."""
    vfx, vfy = pk.foot_ground_velocity(0.2, -0.1, vx=0.1, vy=0.0, omega=0.0)
    xs = []
    for s in np.linspace(0.0, 0.49, 10):   # appui : leg_phase < duty(0.5)
        dx, dy, dz = pk.gait_offset(s, vfx, vfy)
        xs.append(dx)
        assert dz == 0.0                     # pied au sol pendant l'appui
    # dx strictement décroissant → le pied recule, le corps avance
    assert all(b < a for a, b in zip(xs, xs[1:]))


def test_swing_lifts_foot_and_returns():
    """Phase de transfert : le pied se lève (dz>0) puis revient en avant."""
    vfx, vfy = pk.foot_ground_velocity(0.2, -0.1, vx=0.1, vy=0.0, omega=0.0)
    mid = pk.gait_offset(0.75, vfx, vfy)     # milieu du transfert
    assert mid[2] > 0                        # pied levé
    start = pk.gait_offset(0.5, vfx, vfy)[2]
    end = pk.gait_offset(0.999, vfx, vfy)[2]
    assert start == pytest.approx(0.0, abs=1e-3)
    assert end == pytest.approx(0.0, abs=1e-3)


def test_gait_continuity_stance_swing():
    """Continuité de la position horizontale au passage appui→transfert."""
    vfx, vfy = pk.foot_ground_velocity(0.2, -0.1, vx=0.1, vy=0.0, omega=0.0)
    end_stance = pk.gait_offset(0.4999, vfx, vfy)
    start_swing = pk.gait_offset(0.5, vfx, vfy)
    assert end_stance[0] == pytest.approx(start_swing[0], abs=1e-3)
    assert end_stance[1] == pytest.approx(start_swing[1], abs=1e-3)
