# Turtle Scanner - ROS2

## 1. Description du projet

Ce projet a ete realise avec ROS2 et turtlesim.

L'objectif est de faire parcourir l'espace a une tortue scanner (`turtle1`) selon un trajet en serpentin afin de rechercher une tortue cible (`turtle_target`).

Le projet utilise :
- des **topics** (publisher / subscriber)
- des **services** (client / serveur)
- une **interface personnalisee** avec `ResetMission.srv`

---

## 2. Architecture du projet

Le workspace contient deux packages :

### 2.1 `turtle_scanner_luidjy`
Package Python principal contenant les noeuds ROS2 :

- `spawn_target.py`
- `turtle_scanner_node.py`
- `mission_client.py`

### 2.2 `turtle_interfaces`
Package contenant l'interface de service personnalisee :

- `ResetMission.srv`

---

## 3. Role des noeuds

### `spawn_target.py`
Ce noeud utilise le service `/spawn` de turtlesim pour creer une tortue cible appelee `turtle_target` a une position aleatoire.

### `turtle_scanner_node.py`
C'est le noeud principal du projet.

Il permet de :
- recevoir la pose de `turtle1`
- recevoir la pose de `turtle_target`
- generer les waypoints du trajet en serpentin
- deplacer `turtle1`
- detecter la cible
- publier l'etat de detection
- gerer le service `/reset_mission`

### `mission_client.py`
Ce noeud automatise les missions.

Il :
- ecoute `/target_detected`
- appelle `/reset_mission`
- lance automatiquement 3 missions successives

---

## 4. Topics utilises

### `/turtle1/pose`
Type : `turtlesim/msg/Pose`  
Role : donne la position et l'orientation de `turtle1`

### `/turtle_target/pose`
Type : `turtlesim/msg/Pose`  
Role : donne la position et l'orientation de `turtle_target`

### `/turtle1/cmd_vel`
Type : `geometry_msgs/msg/Twist`  
Role : permet de commander le mouvement de `turtle1`

### `/target_detected`
Type : `std_msgs/msg/Bool`  
Role : publie `True` quand la cible est detectee et `False` sinon

---

## 5. Services utilises

### `/spawn`
Type : `turtlesim/srv/Spawn`  
Role : creer une nouvelle tortue dans turtlesim

### `/kill`
Type : `turtlesim/srv/Kill`  
Role : supprimer une tortue existante

### `/reset_mission`
Type : `turtle_interfaces/srv/ResetMission`  
Role : reinitialiser la mission avec une nouvelle cible

---

## 6. Interface personnalisee

### `ResetMission.srv`

```srv
float64 target_x
float64 target_y
bool random_target
---
bool success
string message
