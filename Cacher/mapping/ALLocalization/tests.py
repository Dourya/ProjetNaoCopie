import qi
import argparse
import sys
import time

def main(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")
    localizationProxy = session.service("ALLocalization")

    # Réveille le robot
    posture_service.goToPosture("StandInit", 1.0)
    print("Robot woken up")

    # Get position
    print(localizationProxy.getRobotPosition())

    # S'assurer que le robot est bien immobile
    time.sleep(1)

    # Démarrer l'apprentissage du point de départ
    ret = localizationProxy.learnHome()
    if ret == 0:
        print("Learning OK")
    else:
        print("Error during learning", ret)
        return  # Sortir si échec, inutile d'aller plus loin

    # Déplacer le robot
    motion_service.moveTo(0.5, 0.0, 0.2)

    # Retourner au point de départ
    ret = localizationProxy.goToHome()
    if ret == 0:
        print("Go to home OK")
    else:
        print("Error during go to home", ret)

    # Sauvegarder le panorama
    ret = localizationProxy.save("example")
    if ret == 0:
        print("Saving OK")
    else:
        print("Error during saving", ret)

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
