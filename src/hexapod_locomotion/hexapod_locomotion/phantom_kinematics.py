"""Cinématique exacte de l'hexapode PhantomX (paquet phantomx_description).

Contrairement à un modèle idéalisé, la FK est construite **directement à partir
des repères des joints de l'URDF** (chaîne base_link → c1 → c2 → thigh → tibia →
pied). L'IK est résolue numériquement (moindres carrés amortis) par patte.

Repère de travail : base_link (= MP_BODY), X avant, Y gauche, Z haut.
Joints actionnés (axe local X) : j_c1 (coxa), j_thigh (fémur), j_tibia (tibia).
"""

import math
import numpy as np

# ───────────────────────── Noms de joints (ordre leg_id 0..5) ───────────────
LEG_NAMES = ['rf', 'rm', 'rr', 'lf', 'lm', 'lr']
LEG_JOINT_NAMES = [(f'j_c1_{s}', f'j_thigh_{s}', f'j_tibia_{s}') for s in LEG_NAMES]

JOINT_LIMIT = 2.6179939            # ±150° (limites URDF)

# Pose de repos symétrique et de niveau (déterminée par FK) : pieds tous à la
# même hauteur. base_height permet de poser ces pieds au sol (z monde = 0).
REST_Q = (0.0, -0.8, -1.2)
# Hauteur du corps pour que le bout des pieds (mesh) repose sur le sol (z=0).
BASE_HEIGHT = 0.088

# ───────────────────── Transforms fixes de la chaîne (URDF) ─────────────────
# c1 : position de montage (m) + yaw (rad) dans MP_BODY ; pitch commun.
_C1_MOUNT = {
    'rf': ([0.1248, -0.06164, 0.001116], 0.7854),
    'rm': ([0.0, -0.1034, 0.001116], 0.0),
    'rr': ([-0.1248, -0.06164, 0.001116], -0.7854),
    'lf': ([0.1248, 0.06164, 0.001116], 2.35619),
    'lm': ([0.0, 0.1034, 0.001116], 3.14159),
    'lr': ([-0.1248, 0.06164, 0.001116], 3.92699),
}
_C1_PITCH = 4.7123
# Bout du tibia (pied) dans le repère tibia ≈ longueur du tibia selon +Y.
_FOOT = np.array([0.0, 0.11, 0.0, 1.0])


def _rpy(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _H(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def _trans(xyz):
    return _H(np.eye(3), np.array(xyz, float))


def _rot(rpy):
    return _H(_rpy(*rpy), np.zeros(3))


def _rotx(q):
    return _H(_rpy(q, 0, 0), np.zeros(3))


# Pré-calcul des transforms fixes communs à toutes les pattes.
_C2 = _trans([0, -0.054, 0]) @ _rot([0, 1.5704, 3.14159])
_THIGH = _rot([3.14159, 0, 0])
_TIBIA = _trans([0, -0.0645, -0.0145]) @ _rot([-1.5707, 0, 3.14159])
# Transform de base de chaque patte (base_link → repère du joint c1, avant q1).
_C1_BASE = {
    s: _trans(xyz) @ _rot([0, _C1_PITCH, yaw])
    for s, (xyz, yaw) in _C1_MOUNT.items()
}


def fk(leg_id, q):
    """Cinématique directe : position du pied (m) dans base_link.

    leg_id : 0..5 ; q : (q1, q2, q3) en radians.
    """
    s = LEG_NAMES[leg_id]
    T = (_C1_BASE[s] @ _rotx(q[0]) @ _C2 @ _THIGH @ _rotx(q[1])
         @ _TIBIA @ _rotx(q[2]))
    return (T @ _FOOT)[:3]


def _jacobian(leg_id, q, eps=1e-6):
    J = np.zeros((3, 3))
    f0 = fk(leg_id, q)
    for i in range(3):
        dq = list(q)
        dq[i] += eps
        J[:, i] = (fk(leg_id, dq) - f0) / eps
    return J, f0


def ik(leg_id, target, q_seed=None, iters=40, damping=1e-3, tol=1e-5):
    """IK numérique (moindres carrés amortis) d'une patte.

    target : position désirée du pied (m) dans base_link.
    q_seed : graine (warm-start) pour la continuité ; sinon REST_Q.
    Retourne q (3,) borné aux limites des joints.
    """
    q = np.array(REST_Q if q_seed is None else q_seed, float)
    target = np.asarray(target, float)
    for _ in range(iters):
        J, f = _jacobian(leg_id, q)
        err = target - f
        if np.linalg.norm(err) < tol:
            break
        # dq = J^T (J J^T + λI)^-1 err
        JJt = J @ J.T + damping * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, err)
        q = q + dq
        q = np.clip(q, -JOINT_LIMIT, JOINT_LIMIT)
    return q


# Positions des pieds au repos (base_link), une fois pour toutes.
REST_FEET = [fk(i, REST_Q) for i in range(6)]


# ───────────────────────────── Démarche (repère corps) ──────────────────────
def foot_ground_velocity(rest_x, rest_y, vx, vy, omega):
    """Vitesse du pied au sol (m/s) pour un déplacement corps (vx, vy, omega).

    Sans glissement : v_pied = -(v_corps + omega × r). Le corps avance donc dans
    le sens commandé pendant que les pieds en appui poussent à l'opposé.
    """
    vfx = -(vx - omega * rest_y)
    vfy = -(vy + omega * rest_x)
    return vfx, vfy


def gait_offset(leg_phase, vfx, vfy, duty=0.5, cycle=1.0, step_h=0.04):
    """Décalage (dx, dy, dz) du pied / repos selon la phase de démarche [0,1).

    leg_phase < duty : APPUI (pied au sol, avance à v_pied) ;
    sinon            : TRANSFERT (pied levé, revient en sens inverse).
    """
    t_st = duty * cycle
    if leg_phase < duty:
        s = leg_phase / duty                 # 0 → 1
        k = (s - 0.5) * t_st                 # -t_st/2 → +t_st/2
        return vfx * k, vfy * k, 0.0
    sw = (leg_phase - duty) / (1.0 - duty)   # 0 → 1
    k = (0.5 - sw) * t_st                    # +t_st/2 → -t_st/2
    dz = step_h * math.sin(math.pi * sw)
    return vfx * k, vfy * k, dz
