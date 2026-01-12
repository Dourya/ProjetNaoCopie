import qi
import argparse
import sys
import time
import math

# === Paramètres de la grille ===
GRID_SIZE = 10        # 10x10 cases
CELL_SIZE = 0.2       # chaque case = 0.3 m
SONAR_RANGE = 2.0     # portée max des sonars

# Grille : -1 = inconnu, 0 = libre, 1 = obstacle
grid = [[-1 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

# Position initiale du robot (au centre de la grille)
robot_x = GRID_SIZE // 2
robot_y = GRID_SIZE // 2
robot_theta = 0


def print_grid():
    """Affiche la grille dans la console"""
    for y in range(GRID_SIZE):
        row = ""
        for x in range(GRID_SIZE):
            if x == robot_x and y == robot_y:
                row += "R "
            elif grid[y][x] == -1:
                row += "? "
            elif grid[y][x] == 0:
                row += ". "
            elif grid[y][x] == 1:
                row += "# "
        print(row)
    print("\n")


def update_map_from_sonar(distance, angle_offset):
    angle = robot_theta + math.radians(angle_offset)

    max_dist = int(distance / CELL_SIZE)

    # Cases libres jusqu’à la distance max
    for dist in range(1, max_dist):
        free_x = int(robot_x + math.cos(angle) * dist)
        free_y = int(robot_y + math.sin(angle) * dist)
        if 0 <= free_x < GRID_SIZE and 0 <= free_y < GRID_SIZE:
            grid[free_y][free_x] = 0

    # Si obstacle → marquer la case correspondante
    if distance is not None and distance <= SONAR_RANGE:
        obs_x = int(robot_x + math.cos(angle) * (distance / CELL_SIZE))
        obs_y = int(robot_y + math.sin(angle) * (distance / CELL_SIZE))
        if 0 <= obs_x < GRID_SIZE and 0 <= obs_y < GRID_SIZE:
            grid[obs_y][obs_x] = 1



def main(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    print("Activation des sonars...")
    sonar_service.subscribe("ObstacleDetector")
    time.sleep(1)

    print("Position initiale : assis")
    motion_service.rest()
    time.sleep(2)

    print("Mise en position debout...")
    posture_service.goToPosture("StandInit", 1.0)
    time.sleep(1)

    print("Début de la cartographie...")

    while (1):
        time.sleep(2)
        grid = [[-1 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
        distance_left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
        distance_right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")

        # Mise à jour de la carte
        update_map_from_sonar(distance_left, -10)
        update_map_from_sonar(distance_right, 15)

        # Afficher mesures
        print(f"[Sonar] Gauche : {distance_left if distance_left else 'rien'} m | Droite : {distance_right if distance_right else 'rien'} m")

        # Afficher la carte
        print_grid()

    print("Fin du programme.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.16.1.163", help="Adresse IP du robot")
    parser.add_argument("--port", type=int, default=9559, help="Port NAOqi (par défaut 9559)")
    args = parser.parse_args()

    session = qi.Session()
    try:
        session.connect(f"tcp://{args.ip}:{args.port}")
    except RuntimeError:
        print("Échec de connexion à NAOqi.")
        sys.exit(1)

    main(session)
