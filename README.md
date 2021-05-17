# Projet pour le cours POG

### Étapes du projet

- [x] Définir notre sujet
- [ ] Coder le prototype dans le simulateur
- [ ] Utiliser le lidar et la caméra
- [ ] Utiliser le chien

### Milestones
- Implémentation de fonctions de déplacement (`MoveToPoint` et `MoveUsingPath`)
- Optimisation de l'affichage grâce à `showmap`
- Implémentation du RRT*
- Implémentation du DStar Lite

### To-do 
- Implémentation de l'ajout d'obstacle
- Implémentation de la détection dynamique d'obstacle (ou pas?)
- Implémentation d'une interface graphique (voir les mouvements du chien sur la map en live)
- Incorporation des `automove.py` et `movealiengo.cpp` pour connecter le code au chien-robot 
- Comment intégrer `room` à `Env`? 
- Améliorer l'interface dynamique 

### Questions
- Y a-t-il une boucle entre `lcm.h` et `lcm-cpp.hpp` (dans `unitree_legged_sdk.h`)? Si oui, comment régler ça? 
- Où se trouve `lcm_export.h` ? 
- Problème avec des parties du code de `pthread.h`: comment les régler? Est-ce des fautes? 