import qi
import numpy as np
import matplotlib.pyplot as plt
import time
import math

# Paramètres de connexion au robot
ROBOT_IP = "172.16.1.163"   # ⚠️ adapte avec l'IP du NAO
ROBOT_PORT = 9559

# --- Connexion ---
session = qi.Session()
try:
    session.connect("tcp://{}:{}".format(ROBOT_IP, ROBOT_PORT))
except RuntimeError:
    print("Impossible de se connecter au robot à {}:{}".format(ROBOT_IP, ROBOT_PORT))
    exit(1)

# --- Services ---
sonar = session.service("ALSonar")
memory = session.service("ALMemory")

# Activer les sonars
sonar.subscribe("myApp")

# Paramètres de la map
MAP_SIZE = 20   # 20x20 cellules
CELL_SIZE = 10  # cm par cellule
occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE))

# Position du robot au centre de la map
robot_x, robot_y = MAP_SIZE // 2, MAP_SIZE // 2

def read_sonar():
    """Retourne les distances mesurées (gauche, droite) en mètres"""
    left = memory.getData("Device/SubDeviceList/US/Left/Sensor/Value")
    right = memory.getData("Device/SubDeviceList/US/Right/Sensor/Value")
    return left, right

def update_map(x, y, left, right, grid, max_range=8):
    """
    Met à jour l'occupancy grid avec les mesures sonar gauche/droite.
    Chaque distance est projetée dans la map autour du robot.
    """
    measures = [("gauche", left, math.pi/2), ("droite", right, -math.pi/2)]
    for _, dist, angle in measures:
        if dist > 0 and dist < max_range * (CELL_SIZE/100.0):  # borne max
            r = int(dist / (CELL_SIZE/100.0))  # m -> cellules
            dx = int(math.cos(angle) * r)
            dy = int(math.sin(angle) * r)
            cx, cy = x + dx, y + dy
            if 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE:
                grid[cy, cx] = min(1.0, grid[cy, cx] + 0.8)  # obstacle probable
    return grid

# --- Boucle de mesures ---
try:
    for _ in range(30):  # 30 itérations
        left, right = read_sonar()
        occupancy_grid = update_map(robot_x, robot_y, left, right, occupancy_grid)
        time.sleep(0.5)

finally:
    sonar.unsubscribe("myApp")

# --- Affichage ---
plt.imshow(occupancy_grid, cmap="gray_r", origin="lower")
plt.scatter(robot_x, robot_y, c="red", marker="o")  # robot
plt.title("Occupancy Grid Mapping avec NAO (QI SDK)")
plt.colorbar(label="Probabilité d'occupation")
plt.show()
