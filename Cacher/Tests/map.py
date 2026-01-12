import qi
import argparse
import sys
import time
import os

GRID_WIDTH = 21   # largeur de la grille (impair pour centrer R)
GRID_HEIGHT = 10  # hauteur de la grille
SCALE = 5         # échelle : 1 mètre = 5 cases

def draw_map(distance_left, distance_right):
    # Grille vide
    grid = [["." for _ in range(GRID_WIDTH)] for _ in range(GRID_HEIGHT)]

    # Position du robot (au centre bas de la grille)
    robot_x = GRID_WIDTH // 2
    robot_y = GRID_HEIGHT - 1
    grid[robot_y][robot_x] = "R"

    # Conversion distance -> cases
    if distance_left < 2.0:  # limiter portée
        obs_x = robot_x - 2  # décalé à gauche
        obs_y = robot_y - int(distance_left * SCALE)
        if 0 <= obs_x < GRID_WIDTH and 0 <= obs_y < GRID_HEIGHT:
            grid[obs_y][obs_x] = "M"

    if distance_right < 2.0:
        obs_x = robot_x + 2  # décalé à droite
        obs_y = robot_y - int(distance_right * SCALE)
        if 0 <= obs_x < GRID_WIDTH and 0 <= obs_y < GRID_HEIGHT:
            grid[obs_y][obs_x] = "M"

    # Afficher la grille
    os.system("clear")  # ou "cls" sous Windows
    for row in grid:
        print(" ".join(row))


def main(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    print("Activation des sonars...")
    sonar_service.subscribe("MappingSonar")
    time.sleep(1)

    # Réveil et posture debout
    print("Réveil du robot...")
    motion_service.wakeUp()
    print("Mise en position debout...")
    posture_service.goToPosture("StandInit", 1.0)
    time.sleep(1)

    print("Début du mapping (le robot reste immobile).")

    try:
        while True:
            # Lire distances
            distance_left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            distance_right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")

            draw_map(distance_left, distance_right)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Arrêt manuel.")

    finally:
        sonar_service.unsubscribe("MappingSonar")
        motion_service.rest()
        print("Fin du programme.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.16.1.164",
                        help="Adresse IP du robot")
    parser.add_argument("--port", type=int, default=9559,
                        help="Port NAOqi (par défaut 9559)")

    args = parser.parse_args()
    session = qi.Session()

    try:
        session.connect(f"tcp://{args.ip}:{args.port}")
    except RuntimeError:
        print("Échec de connexion à NAOqi.")
        sys.exit(1)

    main(session)
