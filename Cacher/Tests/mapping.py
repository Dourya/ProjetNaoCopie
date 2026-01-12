import qi
import argparse
import sys
import time


def main(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    print("Activation des sonars...")
    sonar_service.subscribe("ObstacleDetector")
    time.sleep(1)

    # Mettre le robot en position assise
    print("Position initiale : assis")
    motion_service.rest()
    time.sleep(2)

    # Mise en position debout
    print("Mise en position debout...")
    posture_service.goToPosture("StandInit", 1.0)
    time.sleep(1)

    print("Début de la marche...")

    # Position initiale
    position = 0.0
    unit_per_second = 1.0  # correspond à 0.5 m/s réel
    step_time = 0.5
    unit_per_step = unit_per_second * step_time

    try:
        while True:
            # Lire les distances depuis ALMemory
            distance_left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            distance_right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")

            # Affichage des sonars
            if distance_left <= 1.0:
                print(f"Gauche : obstacle à {distance_left:.2f} m")
            else:
                print("Gauche : pas d'obstacle (< 1 m)")

            if distance_right <= 1.0:
                print(f"Droite : obstacle à {distance_right:.2f} m")
            else:
                print("Droite : pas d'obstacle (< 1 m)")

            # Vérification obstacle devant
            distance = min(distance_left, distance_right)
            if distance < 0.3:
                print("⚠️ Obstacle détecté à moins de 0.3 mètre. Arrêt.")
                motion_service.stopMove()
                break

            # Avancer
            motion_service.moveToward(1.0, 0.0, 0.0)
            time.sleep(step_time)

            # Mise à jour de la position
            position += unit_per_step
            print(f"Position actuelle : {position:.2f} unités (réf. départ = 0.0)")

            print("-" * 40)  # séparateur visuel

    except KeyboardInterrupt:
        print("Arrêt manuel.")
        motion_service.stopMove()

    finally:
        sonar_service.unsubscribe("ObstacleDetector")
        print("Fin du programme.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
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