import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from PythonRobotics.PathPlanning.RRTStar.rrt_star import RRTStar

# #plotting the map with imshow
# def showmap(room, dog):
#     dog_x, dog_y = dog.x, dog.y
#     height, width = room.imgdf.shape
#     # human_x, human_y = human.x, human.y

#     fig,ax = plt.subplots(1,figsize=(7,7))
#     fig = plt.plot(dog_x, dog_y, 's')
#     # fig = plt.plot(human_x, human_y, 'x')
    
#     ax.imshow(room.imgdf, origin='lower', interpolation='none', extent=(0, width, 0, height))
#     plt.gca().invert_yaxis()
#     plt.show()
    
# Draw final path
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
    obstacle = []
    for y in range(len(room.imgdf)):
        for x in range(len(room.imgdf.iloc[y])):
            if room.imgdf.iloc[y][x]==1:
                obstacle.append((x+0.5,y+0.5,radius))
    return obstacle

def generatepath(room, dog, goal, obstacle, max_iter=300, search_until_max_iter=False):
    # Set Initial parameters
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
    
#     unit_vector_x = [1,0]
  
    #Récupérer les infos sur le chien
    pos = np.array([dog.x, dog.y])
    curr_angle = dog.rot

    #Définir le vecteur déplacement
    goal = np.array([x,y])
    mov_vector = goal - pos
    
    #Calcul de la distance à parcourir
    dist = np.linalg.norm(mov_vector)

    #Calcul du nouvel angle du chien
    # Largement simplifiable
    dot = np.dot(mov_vector, [1,0])
    det = np.linalg.det(np.array([mov_vector, [1,0]])) # Vecteur x unitaire
    angle = np.degrees(np.arctan2(det, dot)) #problème d'angle hors intervalle parfois
    rot = angle - curr_angle
    rot = np.sign(rot)*(np.abs(rot)%360)
    
    #Debugging
    print("Current X: {}, Current Y: {}".format(pos[0], pos[1]))
    print("Goal X: {}, Goal Y: {}".format(x,y))
    print("Angle absolu {}".format(angle))
    print("Angle relatif {}".format(rot))
    print("Distance à parcourir {}".format(dist))
    print("--------------")
    
    #Voir si on doit ajouter qqch en fct de la speed du chien

    #Appliquer les fonctions du simulateur
    dog.rotate(rot)
    dog.forward(dist)
    
    
#Utilise le path généré pour déplacer le chien
def MoveUsingPath(room, dog, path):
    keypoints = path.copy()
    # keypoints.reverse()

    for x,y in keypoints[1:]:
        MoveToPoint(dog, x, y)
    showmap(room, dog)
    return True