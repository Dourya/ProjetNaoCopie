import qi
import time
import math

def main(session):
    motion_service = session.service("ALMotion")
    
    # Activer le mode de marche
    motion_service.wakeUp()

    # Obtenir la position initiale du robot dans son repère
    start_position = motion_service.getRobotPosition(True)
    print("Position de départ :", start_position)

    # Faire un petit pas en avant (valeurs typiques : x=0.5, durée=1s)
    motion_service.moveToward(1.0, 0.0, 0.0)
    time.sleep(1.0)  # durée du "pas"
    motion_service.moveToward(0.0, 0.0, 0.0)

    # Laisser le robot se stabiliser
    time.sleep(0.5)

    # Obtenir la nouvelle position
    end_position = motion_service.getRobotPosition(True)
    print("Position finale :", end_position)

    # Calcul de la distance parcourue
    dx = end_position[0] - start_position[0]
    dy = end_position[1] - start_position[1]
    distance = math.sqrt(dx**2 + dy**2)

    print("Distance parcourue : {:.3f} mètres".format(distance))

    # Mettre le robot en veille
    motion_service.rest()


if __name__ == "__main__":
    # Connexion au robot
    session = qi.Session()
    try:
        session.connect("tcp://198.18.0.1:9559")  # <-- change l’adresse IP selon ton robot
    except RuntimeError:
        print("Impossible de se connecter au robot.")
        exit(1)

    main(session)
