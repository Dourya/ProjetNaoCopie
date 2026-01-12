import qi
import argparse
import sys
import time
import math

def main(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    print("Activation des sonars...")
    sonar_service.subscribe("ObstacleDetector")
    time.sleep(1)

    # Position initiale
    print("Position initiale : assis")
    motion_service.rest()
    time.sleep(2)

    print("Mise en position debout...")
    posture_service.goToPosture("StandInit", 1.0)
    time.sleep(1)

    # --- Sauvegarde de la position initiale ---
    start_position = motion_service.getRobotPosition(True)
    print("Position initiale :", start_position)

    print("Début de la marche...")

    # --- Initialisation des variables de mesure ---
    start_time = time.time()
    step_count = 0
    step_duration = 0.5  # durée d’un "pas" (en secondes)
    
    try:
        while True:
            # Lire les distances sonar
            distance_left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            distance_right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")
            distance = min(distance_left, distance_right)

            print(f"Distance détectée : {distance:.2f} m")

            if distance < 0.3:
                print("Obstacle détecté à moins de 0.3 mètre. Arrêt.")
                motion_service.stopMove()
                break

            # Avancer pendant un "pas"
            motion_service.moveToward(1.0, 0.0, 0.0)
            time.sleep(step_duration)
            step_count += 1

    except KeyboardInterrupt:
        print("Arrêt manuel.")
        motion_service.stopMove()

    finally:
        # --- Calculs finaux ---
        end_time = time.time()
        total_time = end_time - start_time

        # Position finale et distance parcourue
        end_position = motion_service.getRobotPosition(True)
        dx = end_position[0] - start_position[0]
        dy = end_position[1] - start_position[1]
        distance_moved = math.sqrt(dx**2 + dy**2)

        # --- Affichage des résultats ---
        print("\n======================")
        print("   RÉSULTATS DU TEST  ")
        print("======================")
        print(f"🕒 Temps total de marche : {total_time:.2f} secondes")
        print(f"🚶 Nombre de pas : {step_count}")
        print(f"📏 Distance parcourue : {distance_moved:.3f} m")
        if step_count > 0:
            print(f"➡️ Distance moyenne par pas : {distance_moved / step_count:.3f} m")
        print("======================\n")

        # Nettoyage
        sonar_service.unsubscribe("ObstacleDetector")
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
