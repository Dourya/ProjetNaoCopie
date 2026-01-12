import qi
import argparse
import sys
import time


def main(session):
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    # Activer les capteurs sonars
    print("Activation des capteurs sonars...")
    sonar_service.subscribe("SonarMapping")
    time.sleep(1)

    # Afficher les valeurs des sonars pendant 10 secondes
    print("Affichage des valeurs des capteurs sonars pendant 10 secondes...")
    
    start_time = time.time()
    
    data_left = []
    data_right = []
    while time.time() - start_time < 3:
        # Récupérer les valeurs des capteurs sonars
        left_distance=[]
        right_distance=[]
        left_distance.append(memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value"))
        right_distance.append(memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value"))
        for i in range(1,10):
            left_distance.append(memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value",i))
            right_distance.append(memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value",i))
        data_right.append(right_distance)
        data_left.append(left_distance)

    # Afficher les distances
    # print(f"Front: {front_distance:.2f} m | Left: {left_distance:.2f} m | Right: {right_distance:.2f} m | Back: {back_distance:.2f} m")
    for i in range(len(data_left)):
        print("Wave",i,":\n")
        for j in range(0,10):
            print(f"Left {j}: {data_left[i][j]} m | Right {j}: {data_right[i][j]} m")
            
    # Après 10 secondes, désabonner les capteurs sonars et arrêter
    sonar_service.unsubscribe("SonarMapping")
    print("\nFin de la cartographie. Le programme va s'arrêter.")


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