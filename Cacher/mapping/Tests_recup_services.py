import qi
import argparse
import sys
import time

def main(session):
    # Connexion au service ALWorldRepresentation
    
    try:
        world_rep_service = session.service("ALWorldRepresentation")
    except RuntimeError as e:
        print("Error pour ALWorldRrepresentation :", e)
    try:
        localization_proxy = session.service("ALLocalization")
    except RuntimeError as e:
        print("Error pour ALLocalization :", e)
    try:
        visual_space_history = session.service("ALVisualSpaceHistory")
    except RuntimeError as e:
        print("Error pour ALVisualSpaceHistory :", e)
    try:
        sonar = session.service("ALSonar")
    except RuntimeError as e:
        print("Error pour ALsonar :", e)

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