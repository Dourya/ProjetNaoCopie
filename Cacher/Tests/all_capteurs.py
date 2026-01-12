import qi
import argparse
import sys
import time


def main(session):
    # Créer un service pour ALSonar (qui est plus fiable pour récupérer les valeurs des sonars)
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    # Abonnement aux capteurs sonars
    print("Activation des capteurs sonars...")
    sonar_service.subscribe("SonarMapping")
    time.sleep(1)

    # Afficher les valeurs des capteurs sonars pendant 10 secondes
    print("Affichage des valeurs des capteurs sonars pendant 10 secondes...")
    
    start_time = time.time()
    
    while time.time() - start_time < 10:
        try:
            # Récupérer les valeurs des capteurs sonars en utilisant ALSonar
            front_distance = memory_service.getData("Device/SubDeviceList/US/Front/Sensor/Value")
            left_distance = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            right_distance = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")
            back_distance = memory_service.getData("Device/SubDeviceList/US/Back/Sensor/Value")

            # Afficher les valeurs des capteurs
            print(f"Front: {front_distance:.2f} m | Left: {left_distance:.2f} m | Right: {right_distance:.2f} m | Back: {back_distance:.2f} m")
        
        except RuntimeError as e:
            print("Erreur de récupération des données des capteurs sonars:", e)
        
        # Attendre 1 seconde avant d'afficher à nouveau
        time.sleep(1)

    # Désabonner les capteurs sonars après 10 secondes et arrêter
    sonar_service.unsubscribe("SonarMapping")
    print("\nFin de la cartographie. Le programme va s'arrêter.")


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
