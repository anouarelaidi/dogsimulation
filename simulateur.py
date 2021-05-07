import random
import math
import matplotlib.pyplot as plt
import os
import time
import warnings
import numpy as np
import pandas as pd
from math import sqrt, cos, sin

SKY_RGB = np.array([172, 216, 230])
FLOOR_RGB = np.array([180, 180, 180])
WALL_RGB = np.array([142, 22, 0])
BORDER_RGB = np.array([0, 102, 153])
HUMAN_RGB = np.array([255, 153, 0])

### Intel RealSense intrinsic parameters ###
F = 322.282410  # [distance] : Focal length
F_DEPTH = 0.00188 / 5.32 * 3  # --> Correction factor because focal length is incorrect
U_0 = 320.818268  # [pixels] --> UNUSED
V_0 = 178.779297  # [pixels] --> UNUSED
SIZE_X = 0.0027288  # [m]
SIZE_Y = 0.0015498  # [m]


def bresenham_line(x0, y0, x1, y1):
    if (x0, y0) == (x1, y1):
        return [(x0, y0)]
    elif x0 == x1:
        return [(x0, i) for i in range(min(y0, y1), max(y0, y1) + 1)]

    slope = (y1 - y0) / (x1 - x0)
    coords = [(x, round(slope * (x - x0) + y0)) for x in range(min(x0, x1), max(x0, x1) + 1)]
    return coords


class CameraSim:
    def __init__(self, width, height, u0=None, v0=None, noise=True):
        self.width = width
        self.height = height
        self.f = F
        self.f_depth = F_DEPTH
        self.u_0 = width / 2 if u0 is None else u0
        self.v_0 = height / 2 if v0 is None else v0
        self.density_x = SIZE_X / width
        self.density_y = SIZE_Y / height
        self.noise = noise

    def take_picture(self, mur=None, human=None, width=0, height=0, rot=0.0, posX=0, posY=0):
        """
        :param mur: Liste de murs
        :param width: Largeur de la carte
        :param height: longueur de la carte
        :param rot: Orientation du chien
        :param posX: Coordonnée X du chien sur la carte
        :param posY: Coordonnée Y du chien sur la carte
        :return:
        """
        final_pic = np.zeros([self.height, self.width, 3])
        height_map = np.zeros([self.height, self.width])

        # Draw background
        final_pic[(self.height // 2 + 1):, :, :] = FLOOR_RGB
        final_pic[:(self.height // 2 + 1), :, :] = SKY_RGB

        for pixel_y in range(1, self.height // 2 + 1):
            height_map[self.height // 2 + pixel_y - 1, :] = min(10, self.compute_depth(pixel_y))
        height_map[:(self.height // 2 + 1), :] = np.max(height_map)

        # Draw border walls
        for x in range(-1, width + 1):
            self.draw_block(x - width / 2 + 0.5, height / 2, rot, posX, posY, final_pic, height_map, texture='border')
            self.draw_block(x - width / 2 + 0.5, -1 - height / 2, rot, posX, posY, final_pic, height_map, texture='border')
        for y in range(-1, height + 1):
            self.draw_block(width / 2 + 0.5, y - height / 2, rot, posX, posY, final_pic, height_map, texture='border')
            self.draw_block(-1 - width / 2 + 0.5, y - height / 2, rot, posX, posY, final_pic, height_map, texture='border')

        # Draw inside walls
        for y in range(0, len(mur)):
            for x in mur[y]:
                self.draw_block(x - width / 2 + 0.5, height / 2 - y, rot, posX, posY, final_pic, height_map)

        # Draw human
        if human is not None:
            human.pos()
            self.draw_block(human.x - 0.5, human.y + 0.5, rot, posX, posY, final_pic, height_map,
                            t=0.4, texture='human')


        # Add Gaussian noise
        if self.noise is True:
            height_map = np.maximum(height_map + np.random.normal(scale=0.15, size=(self.height, self.width)),
                                    np.zeros(shape=height_map.shape))

        return final_pic, height_map

    def draw_wall(self, x0, y0, x1, y1, pic, hmap, texture='wall', jitter=True):
        if texture == 'border':
            color = BORDER_RGB + [0, 0, np.random.randint(-20, 20)] if jitter else BORDER_RGB
        elif texture == 'human':
            color = HUMAN_RGB + [np.random.randint(-40, 0), 0, 0] if jitter else HUMAN_RGB
        else:
            color = WALL_RGB + [np.random.randint(-20, 20), 0, 0] if jitter else WALL_RGB

        line = bresenham_line(x0, y0, x1, y1)
        for (x, y) in line:
            if 0 <= x < self.width and 0 <= y < self.height:
                hmap[:y, x] = hmap[y, x]
                if np.array_equal(pic[y, x], FLOOR_RGB):
                    pic[:y, x] = color

    def draw_block(self, x, y, rot, posX, posY, pic, hmap, t=0.5, texture='wall'):
        # Check if point is in front of camera
        _, _, z = self.world_to_cam(x, y, -0.5, rot, posX, posY, 0)
        if z >= 1:
            UL = self.world_to_cam(x - t, y + t, -0.5, rot, posX, posY, 0)
            UR = self.world_to_cam(x + t, y + t, -0.5, rot, posX, posY, 0)
            BR = self.world_to_cam(x + t, y - t, -0.5, rot, posX, posY, 0)
            BL = self.world_to_cam(x - t, y - t, -0.5, rot, posX, posY, 0)

            corners = sorted([UL, UR, BR, BL], key=lambda v: v[2], reverse=True)

            pixel_coords = [self.project(*x) for x in corners]
            self.draw_wall(*pixel_coords[0], *pixel_coords[1], pic, hmap, texture)
            self.draw_wall(*pixel_coords[1], *pixel_coords[3], pic, hmap, texture)
            self.draw_wall(*pixel_coords[0], *pixel_coords[2], pic, hmap, texture)
            self.draw_wall(*pixel_coords[2], *pixel_coords[3], pic, hmap, texture)

    def world_to_cam(self, x, y, z, rot, x0, y0, z0):
        R = np.array([[cos(rot),  -sin(rot),  0],
                      [sin(rot),  cos(rot),   0],
                      [0,         0,          1]])

        x_cam, y_cam, z_cam = R.dot(np.array([x - x0, y - y0, z - z0]))
        return x_cam, -z_cam, y_cam

    def project(self, x, y, z):
        try:
            K = np.array([[self.f,  0,      self.u_0],
                          [0,       self.f, self.v_0],
                          [0,       0,      1]])
            unscaled_point = np.matmul(K, np.array([x, y, z]))

            # Scale back the values to get a vector of the form [u, v, 1]
            scaled_point = np.divide(unscaled_point, unscaled_point[2])
            return round(scaled_point[0]), round(scaled_point[1])
        except Exception as e:
            warnings.warn("Exception caught: " + str(e))
            return 0, 0

    def world_to_pixels(self, x, y, z, rot=0, x0=0, y0=0, z0=0):
        """
        :param x: Coordonnée X d'un point sur la carte
        :param y: Coordonnée Y d'un point sur la carte
        :param z: Coordonnée Z d'un point sur la carte
        :param rot: Orientation du chien
        :param x0: Position X du chien sur la carte
        :param y0: Position Y du chien sur la carte
        :param z0: Position Z du chien sur la carte
        :return:
        """
        x_cam, y_cam, z_cam = self.world_to_cam(x, y, z, rot, x0, y0, z0)
        return self.project(x_cam, y_cam, z_cam)

    def compute_depth(self, y):
        return sqrt((y * self.density_y) ** 2 + self.f_depth ** 2) * (1 / (2 * y * self.density_y) - 1)


class DogRoom:
    def __init__(self, room="default", largeur=40, longueur=120):
        if room[:3] == "map":
            url = "http://www.sig.hec.ulg.ac.be/pog/{}.csv".format(room)
            os.system('wget %s' % url)
            # !wget -N http://www.sig.hec.ulg.ac.be/pog/map01.csv
        if room[:4] == "alea":
            self.mur = [[] for _ in range(longueur)]
            pc = int(room[4:])
            nb = int(pc * largeur * longueur / 100)
            liste = list(range(largeur * longueur))
            random.shuffle(liste)
            for i in range(nb):
                l = liste[i] // largeur
                c = liste[i] % largeur
                self.mur[l].append(c)
            # remarque: aucune garantie d'avoir un passage entre deux points
        elif room != "default":
            if room[-4:] != ".csv":
                room += ".csv"
            self.mur = []
            self.imgdf = pd.read_csv(room, header=None, sep=';') #crée le DataFrame pour l'imshow
            fok = True
            f = open(room, "r")
            largeur = -1
            longueur = 0
            for ligne in f:
                liste = ligne.split(";")
                if largeur < 0:
                    largeur = len(liste)
                if largeur != len(liste):
                    print("Erreur: La largeur de la salle n'est pas constante")
                    print("Ligne ", longueur, " ignoree")
                else:
                    self.mur.append([])
                    for i in range(largeur):  
                        if liste[i].strip() != "0":
                            self.mur[longueur].append(i)
                    longueur += 1
            f.close()
        else:
            self.mur = [[] for _ in range(longueur)]

        self.largeur = largeur
        self.longueur = longueur

    def plotmap(self, couleur="black", chien=None):
        print("Dimension: {} x {}".format(self.longueur, self.largeur))
        x = []
        y = []
        mid = -self.largeur / 2
        midy = -self.longueur / 2
        for i in range(self.largeur):
            x.append(i + mid)
            y.append(-1 + midy)
            x.append(i + mid)
            y.append(self.longueur + midy)
        for i in range(self.longueur):
            x.append(-1 + mid)
            y.append(self.longueur - 1 - i + midy)
            x.append(self.largeur + mid)
            y.append(i + midy)
            for j in self.mur[i]:
                x.append(j + mid)
                y.append(self.longueur - 1 - i + midy)
        for i in range(len(x)):
            xp = [x[i], x[i] + 1, x[i] + 1, x[i], x[i]]
            yp = [y[i], y[i], y[i] + 1, y[i] + 1, y[i]]
            plt.plot(xp, yp, color=couleur)

        if chien != None:
            plt.scatter([chien.x + mid], [self.longueur - 1 - chien.y + midy])
            if chien.humanF:
                chien.human.pos()
                plt.scatter([chien.human.x + mid], [self.longueur - 1 - chien.human.y + midy])

        plt.axis('equal')
        plt.show()

    def help(self):
        print("DOGROOM")
        print("dogroom(room='default',largeur=40,longueur=120)")
        print(
            "   Avertissement: cette fonction ne peut être utilisée que pour du debugging et PAS dans la version finale du script")
        print("   room='default' -> piece vide de dimension largeur x longueur")
        print("   room='aleapp' -> piece de dimension largeur x longueur avec pp% de blocs à l'intérieur")
        print("   room='map01' -> une carte prédéfinie pour vous (01->04 actuellement)")
        print("   room='nomfichier.csv' -> lecture de la config de la pièce dans un fichier csv")
        print("--------")
        print("plotmap(couleur='black') -> affichage de la carte de la pièce")


#########################################################
class Human:
    def __init__(self, room, speed=0.4, duree=3600, image=[], x=-1, y=0, rot=0):
        self.x = x
        self.y = y
        self.rot = rot * math.pi / 180
        self.speed = speed
        self.image = image
        self.duree = duree
        self.lastime = time.time()
        self.initime = self.lastime
        self.room=room
        self.cptmv = 0
        if self.x < 0:
            # mylist=range(0,room.largeur*room.longueur)
            # random.shuffle(mylist)
            while (True):
                # i=mylist.pop(0)
                # self.x=mylist[i]//room.largeur
                # self.y=mylist[i]%room.largeur
                self.x = random.randrange(0, room.largeur)
                self.y = random.randrange(0, room.longueur)
                if not (self.x in room.mur[self.y]):
                    break

    def pos(self):
        t = time.time()
        if t <= self.initime + self.duree:
            for i in range(0, int((t - self.lastime) / 2)):  # un mouvement toutes les 2 secondes
                self.cptmv += 1
                if self.cptmv == 5:
                    self.rot = random.random() * 2 * math.pi
                    self.cptmv = 0
                x, y = self.x, self.y
                encore = True
                while encore:
                    self.x += self.speed * math.cos(self.rot)
                    self.y -= self.speed * math.sin(self.rot)
                    if int(self.x) in self.room.mur[int(self.y)]:
                        self.x, self.y = x, y
                        self.rot = random.random() * 2 * math.pi
                    else:
                        encore = False
            self.lastime = t


#########################################################
class Dog:
    def __init__(self, room, x=-1, y=0, speed=0.5, rot=0):
        self.largeur = room.largeur
        self.longueur = room.longueur
        self.x = x
        self.y = y
        self.rot = rot * math.pi / 180
        self.speed = speed
        self.humanF = False

        if self.x < 0:
            self.x = self.largeur // 2
            self.y = self.longueur // 2
        if self.x in room.mur[self.y]:
            room.mur[self.y].pop(self.x)
        self.room = room
        self.cam = CameraSim(640, 480)
        # self.distmax = (self.largeur ** 2 + self.longueur ** 2) ** 0.5
        self.distmax = math.hypot(self.largeur, self.longueur) #Test d'optimisation

    def pos(self):
        print("Position: ({},{})  - Rotation: {}".format(self.x, self.y, self.rot))

    def forward(self, direction=1):
        x = self.x
        y = self.y
        ok = True

        newdist = (0.975 + random.random() / 20) * self.speed
        newangle = (0.975 + random.random() / 20) * self.rot * math.pi / 180
        self.x += direction * newdist * (1 + random.random() / 10) * math.cos(newangle)
        self.y -= direction * newdist * (1 + random.random() / 10) * math.sin(newangle)

        if self.x < 0:
            print("BOUM: mur gauche")
            ok = False
        elif self.x >= self.largeur:
            print("BOUM: mur de droite")
            ok = False
        if self.y < 0:
            print("BOUM: mur du haut")
            ok = False
        elif self.y >= self.longueur:
            print("BOUM: mur du bas")
            ok = False
        if int(self.x) in self.room.mur[int(self.y)]:
            print("BOUM: obstacle!")  # s'il tombe pile dessus
            ok = False
        if not ok:
            self.x = x
            self.y = y

        return ok

    def backward(self):
        return self.forward(-1)

    def sidestep(self, direction=1):
        self.rotate(90)
        self.forward(direction)
        self.rotate(-90)

    def rotate(self, angle):
        if angle < -360 or angle > 360:
            print("Erreur: angle hors intervalle")
            return

        self.rot += angle * (0.99 + random.random() / 50)
        if self.rot < 0:
            self.rot += 360
        elif self.rot > 360:
            self.rot -= 360

    def attitude(self, cmd, val):
        print("Le chien vient d'effectuer la commande {} avec une intensité de {}".format(cmd, val))

    def angle0360(self, angle):
        if angle < 0:
            angle += 360
        elif angle > 360:
            angle -= 360
        return angle

    def computeangle(self, x, y):
        if abs(x) < 0.001:
            if y >= 0:
                a = math.pi / 2
            else:
                a = math.pi * 1.5
        else:
            a = math.atan(y / x)
            if x < 0:
                a += math.pi
            elif y < 0:
                a += 2 * math.pi
        return a * 180 / math.pi

    def newdist2(self, x, y, angle):
        # warning one coordinate corresponds to the top corner of a unit square!
        dist = self.distmax
        refangle = self.angle0360(self.rot + angle)
        if y + 1 < self.y:  # bloc au-dessus
            # check face inf
            a1 = self.computeangle((x + 1) - self.x, self.y - (y + 1))
            a2 = self.computeangle(x - self.x, self.y - (y + 1))
            if a1 <= refangle <= a2:
                dist = min((self.y - y - 1) / math.sin(refangle * math.pi / 180), dist)
            # print("({} ; {} ) - [{} ; {}] {} {}".format(x,y,a1,a2,(x+1)-self.x,y+1-self.y))
        elif self.y < y:  # bloc en-dessous
            # check face sup
            a1 = self.computeangle(x - self.x, self.y - y)
            a2 = self.computeangle((x + 1) - self.x, self.y - y)
            if a1 <= refangle <= a2:
                dist = min((self.y - y) / math.sin(refangle * math.pi / 180), dist)
        a = []
        if x > self.x:  # bloc a droite-check face gauche
            a = [x - self.x, self.y - (y + 1), self.y - y]
        elif x + 1 < self.x:  # bloc a gauche-check face droite
            a = [x + 1 - self.x, self.y - y, self.y - (y + 1)]

        if len(a) > 0:
            a1 = self.computeangle(a[0], a[1])
            a2 = self.computeangle(a[0], a[2])
            if x > self.x and a[1] * a[2] < 0:  # ouch a cheval->2quadrants
                if refangle >= a1 or refangle <= a2:
                    dist = min(a[0] / math.cos(refangle * math.pi / 180), dist)
            elif a1 <= refangle <= a2:
                dist = min(a[0] / math.cos(refangle * math.pi / 180), dist)

        return dist

    def lidar(self, angle):
        if angle < -360 or angle > 360:
            print("Erreur: angle hors intervalle")
            return 0

        # considerons la limite
        dist1 = dist2 = self.distmax
        newangle = self.angle0360(self.rot + angle) * math.pi / 180
        if 0 < newangle < math.pi:  # top limit
            dist1 = self.y / math.sin(newangle)
        elif newangle > math.pi:  # bottom limit
            dist1 = (self.y - self.longueur) / math.sin(newangle)
        if newangle < math.pi / 2 or newangle > math.pi * 1.5:  # right limit
            dist2 = (self.largeur - self.x) / math.cos(newangle)
        elif math.pi / 2 < newangle < math.pi * 1.5:  # left limit
            dist2 = -self.x / math.cos(newangle)
        dist = min(dist1, dist2)

        for i in range(self.longueur):
            for j in self.room.mur[i]:
                dist = min(self.newdist2(j, i, angle), dist)

        if self.humanF:
            self.human.pos()
            dist = min(self.newdist2(self.human.x - 0.5, self.human.y + 0.5, angle), dist)

        return dist

    def plotlidar(self, precision=1, room=False):
        x = []
        y = []
        for a in range(0, 360, precision):
            d = self.lidar(a)
            # print(a,d,d*math.cos(a*math.pi/180),d*math.sin(a*math.pi/180))
            if d < self.distmax:
                x.append(d * math.cos(a * math.pi / 180))
                y.append(d * math.sin(a * math.pi / 180))
        plt.scatter(x, y, s=1)
        plt.scatter([0], [0])
        plt.axis('equal')
        if room:
            self.room.plotmap('yellow')
        plt.show()

    def plotmap(self):
        self.room.plotmap('black', self)

    def human(self, speed=0.4, duree=3600, image='none'):
        self.human_var = self.human = Human(self.room, speed, duree, image)
        self.humanF = True

    def get_human(self):
        return self.human_var

    def tag(self, image, x, y):
        print("Not yet available")

    def camera(self):
        return self.cam.take_picture(self.room.mur, self.get_human(), width=self.room.largeur, height=self.room.longueur,
                                     rot=-self.rot * math.pi / 180 + math.pi / 2, posX=self.x - self.room.largeur / 2,
                                     posY=self.room.longueur / 2 - self.y)

    def plotRGB(self):
        pic, _ = self.camera()
        plt.imshow(pic.astype('uint8'))
        plt.show()

   
    def plotdepth(self):
        _, depth = self.camera()
        plt.imshow(depth, cmap="gray", vmin=np.min(depth), vmax=np.max(depth))
        plt.show()
        

    def help(self):
        print("DOG")
        print("--Création")
        print("dog(room,x=-1,y=0,speed=0.5,rot=0)")
        print("    room: pièce créée par appel à dogroom()")
        print("    x,y,speed,rot: position initiale et vitesse par défaut")
        print("    x=-1 => au centre de la pièce.  On suppose position libre")
        print("--------")
        print("--Déplacement - attention marge d'erreur aléatoire pour chaque mouvement")
        print("forward(coefficientspeed=1)")
        print("    Pour faire avancer le chien droit devant lui.")
        print("    coefficientspeed: multiplicateur de la vitesse par défaut.  Marche arrière si négatif")
        print("backward()")
        print("    Marche arrière <=> forward(-1)")
        print("sidestep(coefficientspeed=1)")
        print(
            "    Déplacement latéral avec le coefficient coefficientspeed.  A gauche si positif.  A droite si négatif")
        print("rotate(angle)")
        print("    Rotation de angle degrés [-360;360] sur place.")
        print("attitude(commande, valeur)")
        print("    Cette fonction sera améliorée ultérieurement.")
        print("    commande:  roll, yaw, pitch, height, led")
        print("    valeur: intensité de la commande")
        print("--------")
        print("--Status-capteurs")
        print("pos()")
        print("    Position x,y et angle rotation actuels")
        print(
            "    Cette fonction ne peut être utilisée que pour du debugging et PAS en mode 'production' dans la version finale du programme")
        print("lidar(angle)")
        print(
            "    Indique la distance au plus proche objet dans la direction angle par rapport à l'orientation actuelle du chien")
        print("plotlidar(precision=1)")
        print("    Affiche une carte avec les distances des obstacles visibles les plus proches")
        print("    => appel à lidar(angle) avec angle de 0 à 360 par pas de 'precision'")
        print("plotRGB()")
        print("    Photo de ce qui est en face du chien")
        print("plotdepth()")
        print("    Image représentant ce qui est en face du chien.  Chaque couleur représente une distance au plus proche objet")
        print("camera()")
        print("    Retourne une liste de deux éléments.  Chaque élément est une matrice (liste de listes) de 640x480")
        print(
            "    Elem1: image 640 pixels par 480px de la caméra.  Chaque élément est une liste des trois intensités RGB")
        print("    Elem2: distance par rapport au chien des 640x480 pixels")
        print("--------")
        print("--Interactions")
        print(
            "-- Note: une image est une liste (ligne/hauteur) de listes (colonnes/largeur) de listes (couleurs R,G,B entre 0 et 255)")
        print("human(speed=0.4,duree=5,image='none')")
        print(
            "    Un individu est ajouté dans la carte et se déplace aléatoirement à une vitesse 'speed' pendant 'duree' minutes")
        print("    Une image peut être associée à l'individu.")
        print("tag(image,x,y)")
        print("    Une image peut être placée à la position (x,y)")


###################################################