# turtle_scanner_luidjy

## Partie 1 - Spawn d'une cible aleatoire

Dans cette partie, on a cree le noeud `spawn_target.py`.
Ce noeud utilise le service `/spawn` de turtlesim pour faire apparaitre une tortue appelee `turtle_target` a une position aleatoire.

### Resultat dans TurtleSim

![Spawn target](images/spawn_target.png)

| Kp_ang | Kp_lin | Observation |
|--------|--------|-------------|
| 5.0 | 0.8 | Lent, petit temps d'attente au virage, quelques zigzags, mais s'arrete |
| 20.0 | 5.0 | Trop rapide, rotation excessive, boucle, ne s'arrete plus |
| 1.0 | 20.0 | Fait des cercles, ne s'arrete plus |
| 5.0 | 1.0 | Comportement le plus stable, bon resultat |
