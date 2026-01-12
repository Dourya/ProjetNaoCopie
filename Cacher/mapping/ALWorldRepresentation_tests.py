import qi
import argparse
import sys
import time

def main(session):
    # Connexion au service ALWorldRepresentation
    world_rep_service = session.service("ALWorldRepresentation")

    print("Récupération des objets connus dans l'environnement...")

    for i in range(0,100):
        world_rep_service

    try:
        # Récupérer la liste des objets détectés (objets connus dans le monde)
        objects = world_rep_service.getObjects()
        
        if not objects:
            print("Aucun objet détecté pour le moment.")
        else:
            print(f"{len(objects)} objet(s) détecté(s) :")
            for obj in objects:
                # Chaque objet est un dict avec des infos sur l'objet
                print(f"- Objet ID : {obj['id']}")
                print(f"  Type : {obj.get('type', 'N/A')}")
                print(f"  Position : {obj.get('position', 'N/A')}")
                print(f"  Taille : {obj.get('size', 'N/A')}")
                print(f"  Confiance : {obj.get('confidence', 'N/A')}")
                print()
                
    except RuntimeError as e:
        print("Erreur lors de la récupération des objets :", e)

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