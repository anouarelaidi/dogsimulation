import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from PythonRobotics.PathPlanning.RRTStar.rrt_star import RRTStar

def showmap(room, dog = None, human = None, goal = None, obstacle = None, path = None):
    """
    Affiche la carte
    room: objet de classe DogRoom (obligatoire)
    dog: objet de classe Dog
    human: objet de classe Human
    goal: array des coordonnées du point destination [x,y]
    obstacle: array des tuples des obstacles (x,y,radius)
    path: array des points intermédiaires sur le chemin [[x1,y1],[x2,y2],...]
    """
    fig,ax = plt.subplots(1,figsize=(7,7))
    width, height = room.largeur, room.longueur

    if dog is not None:
        plt.plot(dog.x, dog.y, 'o', color='blue')
    if human is not None:
        plt.plot(human.x, human.y, 's', color='red')
    if path is not None:
        plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--') #Plotting the path
        plt.plot([x for (x, y) in path], [y for (x, y) in path], 'o', color='green') #Plotting the path points
    if goal is not None:
        plt.plot(goal[0],goal[1],'x', color='orange')
    if obstacle is not None:
        for (x, y, radius) in obstacle:
            cir = plt.Circle((x, y), radius, color='r', alpha=0.3, fill=True)
            ax.add_patch(cir)

    ax.imshow(room.imgdf, origin='lower', interpolation='none', extent=(0, width, 0, height))
    plt.gca().invert_yaxis()
    plt.show()

def generateObstacle(room, radius):
    """
    Génère des obstacles basé sur la carte du Simulateur.
    Les obstacles sont de format (x, y, rayon).
    room: Objet DogRoom,
    radius: rayon des obstacles
    """
    obstacle = []
    for y in range(room.longueur):
        for i in range(len(room.mur[y])):
            x = room.mur[y][i]
            obstacle.append((x+0.5,y+0.5,radius))
    return obstacle

def generatepath(room, dog, goal, obstacle, max_iter=300, search_until_max_iter=False):
    #Fonction utile?
    rrt_star = RRTStar(
        start=[dog.x, dog.y],
        goal=goal,
        rand_area=[-100, 100],
        obstacle_list = obstacle,
        expand_dis=30.0,
        path_resolution=2.0,
        goal_sample_rate=20,
        connect_circle_dist=50.0,
        max_iter=max_iter,
        search_until_max_iter=search_until_max_iter)
    path = rrt_star.planning(animation=False)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
    return path

def MoveToPoint(dog, x, y):

    #Récupérer la position et l'angle du chien
    pos = np.array([dog.x, dog.y])
    curr_angle = dog.rot

    #Définir le vecteur déplacement
    goal = np.array([x,y])
    mov_vector = goal - pos

    #Calcul de la distance à parcourir
    dist = np.linalg.norm(mov_vector)

    #Calcul du nouvel angle du chien
    # Sûrement largement simplifiable, mais tjrs garder le arctan2 je pense
    dot = np.dot(mov_vector, [1,0])
    det = np.linalg.det(np.array([mov_vector, [1,0]])) # Vecteur x_unitaire
    angle = np.degrees(np.arctan2(det, dot))
    rot = angle - curr_angle
    rot = np.sign(rot)*(np.abs(rot)%360) #Pour résoudre les problèmes d'angle hors intervalle admissible

    #Debugging
    print("Current X: {}, Current Y: {}".format(pos[0], pos[1]))
    print("Goal X: {}, Goal Y: {}".format(x,y))
    print("Angle absolu {}".format(angle))
    print("Angle relatif {}".format(rot))
    print("Distance à parcourir {}".format(dist))
    print("--------------")

    #Applique les fonctions du simulateur
    dog.rotate(rot)
    dog.forward(dist)


#Utilise le path généré pour déplacer le chien
def MoveUsingPath(room, dog, path, reverse=False, limit=None):
    """
    Déplace un chien en suivant un path (Array de keypoints intermédiaires)
    """
    keypoints = path.copy()
    if reverse:
        keypoints.reverse()
    for i, (x, y) in enumerate(keypoints[1:], start=1):
        MoveToPoint(dog, x, y)
        if i == limit:
            break
    showmap(room, dog)
    return True