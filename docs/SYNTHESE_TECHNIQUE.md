# Projet 3 — Faire marcher l'hexapode 🕷️

> Robotique avancée — M2 Expert Mécatronique — YNOV

## En une phrase

On a créé un programme qui **transforme une consigne simple** (« avance »,
« tourne ») **en mouvements coordonnés des 6 pattes** d'un robot hexapode,
le tout visualisé en 3D.

---

## L'idée générale

On donne au robot un ordre du type **« va à telle vitesse, dans telle
direction »**. Le programme se charge de **calculer comment bouger chaque patte**
pour que le robot se déplace réellement, comme on le demande.

C'est exactement comme un chef d'orchestre : on lui dit *« joue plus vite »*, et
lui sait quoi faire faire à chaque musicien.

---

## Le schéma (du clavier jusqu'au robot)

```
   ⌨️  CLAVIER                    🧠 CERVEAU DU ROBOT                 👁️  ÉCRAN 3D
 (téléopération)                  (nœud locomotion)                    (RViz)

  ┌───────────┐   « avance     ┌──────────────────────┐   angles    ┌──────────┐
  │  Touches  │   à 0,1 m/s »  │  1. Reçoit l'ordre   │   des 18    │          │
  │ Z Q S D   │ ─────────────▶ │  2. Choisit où poser │   moteurs   │  Le      │
  │ A E ␣ + - │   (message     │     chaque patte     │ ──────────▶ │  PhantomX│
  └───────────┘    "Twist")    │  3. Calcule les      │             │  marche  │
                                │     angles (maths)   │             │  sur le  │
                                │  4. Fait "avancer"   │   position  │  sol     │
                                │     le robot         │ ──────────▶ │          │
                                └──────────────────────┘  du corps   └──────────┘
```

**3 acteurs :**
- ⌨️ **Le clavier** envoie une consigne de vitesse (un message standard appelé *Twist*).
- 🧠 **Le cerveau** (notre programme principal) transforme cette consigne en
  mouvements de pattes.
- 👁️ **L'écran 3D** (RViz) affiche le robot PhantomX en train de marcher.

---

## Ce que fait le « cerveau », étape par étape

À chaque instant (≈ 30 fois par seconde), le programme :

1. **écoute la consigne** : à quelle vitesse, dans quelle direction, et faut-il tourner ?
2. **décide où poser les pattes** : la moitié des pattes pousse le sol vers
   l'arrière (ce qui fait avancer le corps), pendant que l'autre moitié se lève
   et se replace devant. Puis on alterne. C'est la **marche « tripode »**, la
   démarche naturelle des insectes.
3. **calcule les angles des moteurs** : pour amener le bout de la patte à
   l'endroit voulu, il faut trouver les bons angles des 3 articulations. C'est
   un calcul de géométrie (la « cinématique inverse »).
4. **fait avancer le robot dans la scène** : on additionne les petits
   déplacements pour que le corps se déplace vraiment sur le sol.

---

## Les points clés qu'on peut mettre en avant

🦿 **Un vrai modèle 3D** — on utilise le PhantomX, un hexapode réel (6 pattes,
3 articulations chacune), avec ses vraies pièces 3D.

🧭 **Le robot marche dans le bon sens** — quand on dit « avance », les pattes
poussent vers l'arrière et le corps va bien vers l'avant (vérifié par la mesure).

🌍 **Le robot se déplace pour de vrai** — il ne reste pas sur place : il avance,
recule et tourne sur un sol, à la bonne hauteur (les pieds touchent le sol).

✅ **C'est testé** — 11 tests automatiques vérifient que les calculs et la marche
sont corrects.

---

## La commande : un seul message pour tout piloter

Tout passe par un message standard `TwistStamped` (le sujet l'imposait) :

| On écrit dans… | …et le robot |
|----------------|--------------|
| `linear.x` | avance / recule |
| `linear.y` | se décale sur le côté |
| `angular.z` | tourne sur lui-même |

---

## Pour le voir tourner

```bash
source install/setup.bash
ros2 launch hexapod_locomotion teleop.launch.py
```
Puis on pilote au clavier : **Z** avancer, **S** reculer, **Q/D** se décaler,
**A/E** tourner, **Espace** stop.

---

## En résumé pour le prof

> « On a un robot hexapode 3D. On lui envoie une consigne de vitesse toute
> simple. Notre programme calcule en temps réel les angles des 18 moteurs pour
> produire une marche d'insecte (tripode), et fait avancer le robot dans la
> bonne direction sur un sol. Tout est piloté par un seul message ROS standard,
> et validé par des tests automatiques. »
