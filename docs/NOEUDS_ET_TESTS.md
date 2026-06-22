# Les nœuds ROS & comment les tester (sans tout lancer)

> Projet 3 — Déplacement de l'hexapode. Guide pratique.

Ce document explique **simplement** : (1) les nœuds ROS et leur rôle, (2) comment
tester un nœud seul, (3) comment tester les fonctions IK / FK sur une patte.

---

## 1. C'est quoi un « nœud » ?

Un **nœud** = un petit programme indépendant. Les nœuds se parlent en
**publiant** et en **écoutant** des messages sur des **topics** (des « canaux »).

Dans notre projet, 4 nœuds collaborent :

```
 teleop_key_node ──/cmd_vel──▶ locomotion_node ──/joint_states──▶ robot_state_publisher ──▶ RViz
   (clavier)                      (cerveau)        │                  (calcul TF pattes)
                                                   └──/tf (odom→base_link)──▶ RViz
 world_node ──/environment (le sol)─────────────────────────────────────────▶ RViz
```

| Nœud | Ce qu'il écoute | Ce qu'il publie | Rôle |
|------|-----------------|-----------------|------|
| **teleop_key_node** | le clavier | `/cmd_vel` | transforme les touches en consigne de vitesse |
| **locomotion_node** | `/cmd_vel` | `/joint_states` + `/tf` | **le cerveau** : calcule les angles des pattes et fait avancer le robot |
| **world_node** | rien | `/environment` | affiche le sol |
| **robot_state_publisher** | `/joint_states` + URDF | `/tf` des pattes | place chaque pièce du robot en 3D |

> 💡 L'avantage de ROS : chaque nœud peut tourner **seul**. On peut donc tester
> le « cerveau » sans clavier, sans RViz, juste en lui envoyant des messages à la main.

---

## 2. Tester un nœud SEUL (sans la simu complète)

### Préparation (à faire dans chaque terminal)
```bash
cd ~/Bureau/RobotiqueAvancee/Projet3_Deplacement
source install/setup.bash
```

### Exemple : tester le cerveau (`locomotion_node`) tout seul

**Terminal 1 — on lance juste le nœud :**
```bash
ros2 run hexapod_locomotion locomotion_node
```

**Terminal 2 — on lui envoie une consigne « avance » à la main :**
```bash
ros2 topic pub -r 30 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.08}}}'
```
(pour tourner : `'{twist: {angular: {z: 0.4}}}'`)

**Terminal 3 — on regarde ce que le nœud produit :**
```bash
# Les angles calculés pour les 18 moteurs :
ros2 topic echo /joint_states

# Le déplacement du robot (sa position qui avance) :
ros2 run tf2_ros tf2_echo odom base_link
```
👉 On doit voir les angles bouger, et la position (`Translation`) de `base_link`
augmenter : le cerveau fonctionne, **sans avoir lancé RViz ni le clavier**.

### Outils utiles pour inspecter (sans rien coder)
```bash
ros2 node list                 # quels nœuds tournent
ros2 topic list                # quels canaux existent
ros2 topic hz /joint_states    # à quelle fréquence le nœud publie
ros2 topic info /cmd_vel       # qui écoute / qui publie ce topic
```

---

## 3. Tester les fonctions IK et FK sur UNE patte

Les calculs de patte sont dans le module **`phantom_kinematics.py`**. Ils ne
dépendent pas de ROS → on peut les tester directement en Python.

- **FK** (cinématique *directe*) : *« si je mets ces angles, où est le pied ? »*
- **IK** (cinématique *inverse*) : *« je veux le pied ICI, quels angles ? »*

Les pattes sont numérotées **0 à 5** : `0=rf` (avant-droit), `1=rm`, `2=rr`,
`3=lf`, `4=lm`, `5=lr`.

### Test rapide en ligne de commande
```bash
source install/setup.bash
python3 - <<'PY'
from hexapod_locomotion import phantom_kinematics as pk

leg = 0   # patte avant-droite (rf)

# --- FK : des angles -> position du pied ---
angles = (0.0, -0.8, -1.2)            # (coxa, fémur, tibia) en radians
pied = pk.fk(leg, angles)
print("FK  : angles", angles, "-> pied (x,y,z) =", pied.round(3), "m")

# --- IK : une position -> les angles ---
cible = pied                          # on reprend la même position
angles_calcules = pk.ik(leg, cible)
print("IK  : pied", cible.round(3), "-> angles =", angles_calcules.round(3))

# --- Vérif : on refait la FK avec les angles trouvés ---
verif = pk.fk(leg, angles_calcules)
erreur = ((verif - cible) ** 2).sum() ** 0.5
print("Vérif: erreur de placement =", round(erreur * 1000, 3), "mm  (doit être ~0)")
PY
```
Sortie attendue : l'IK retrouve des angles qui replacent le pied au bon endroit
(erreur de quelques millièmes de mm).

### Bouger le pied de 3 cm vers l'avant
```bash
python3 - <<'PY'
from hexapod_locomotion import phantom_kinematics as pk
leg = 0
pied = pk.fk(leg, (0.0, -0.8, -1.2))
cible = (pied[0] + 0.03, pied[1], pied[2])   # +3 cm en X
angles = pk.ik(leg, cible)
print("Pour avancer le pied de 3 cm, angles =", angles.round(3))
print("Position obtenue :", pk.fk(leg, angles).round(3))
PY
```

### Lancer les tests automatiques (11 tests)
```bash
source install/setup.bash
python3 -m pytest src/hexapod_locomotion/test/test_phantom_kinematics.py -v
```
Ils vérifient notamment : l'IK retombe bien sur la cible, les angles restent
dans les limites des moteurs, la pose de repos est symétrique, et la marche va
dans le bon sens.

---

## 4. Récapitulatif des commandes

| Je veux… | Commande |
|----------|----------|
| Lancer un seul nœud | `ros2 run hexapod_locomotion <nœud>` |
| Envoyer une consigne | `ros2 topic pub -r 30 /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.08}}}'` |
| Voir les angles | `ros2 topic echo /joint_states` |
| Voir le déplacement | `ros2 run tf2_ros tf2_echo odom base_link` |
| Tester FK/IK d'une patte | script Python ci-dessus (`pk.fk` / `pk.ik`) |
| Lancer tous les tests | `python3 -m pytest src/hexapod_locomotion/test/ -v` |
| Tout lancer (démo) | `ros2 launch hexapod_locomotion teleop.launch.py` |
