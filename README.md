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

### Questions
- Qu'est-ce qui ne fonctionne pas avec `unitree_legged_sdk`?
- Pourquoi cela ne suffit-il pas de rajouter la classe `DogRoom` dans `dstarlite.py` pour qu'il comprenne ce que veut dire `room`? 