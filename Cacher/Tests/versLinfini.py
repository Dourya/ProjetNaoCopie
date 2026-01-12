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

    # Réveil du robot
    motion_service.wakeUp()

    # Mise en position debout
    print("Mise en position debout...")
    posture_service.goToPosture("StandInit", 1.0)
    time.sleep(1)

    print("Début de la marche...")

    try:
        while True:
            # Lire les distances
            distance_left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            distance_right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")

            distance = min(distance_left, distance_right)
            print(f"Distance détectée : {distance:.2f} m")

            if distance < 0.3:
                print("Obstacle détecté ! Arrêt et rotation à droite.")
                motion_service.stopMove()

                # Tourner à droite (rotation = -90°)
                motion_service.moveTo(0.0, 0.0, -1.57)  # -1.57 rad ≈ -90°
                time.sleep(1)

            else:
                # Avancer
                motion_service.moveToward(0.5, 0.0, 0.0)
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("Arrêt manuel.")
        motion_service.stopMove()

    finally:
        sonar_service.unsubscribe("ObstacleDetector")
        motion_service.rest()
        print("Fin du programme.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.16.1.163",
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
