#! /usr/bin/env python
# main.py
# -*- encoding: UTF-8 -*-

import argparse
import sys
import os
import qi
import time
import signal
import cv2
import numpy as np

from movement.navigation import run_algo_on_robot

import json
# Assurez-vous d'importer la nouvelle fonction (supposons qu'elle est dans navigation.py)
from movement.navigation import drive_robot_with_algo

from tensorflow.keras.models import load_model

from movement import wake_up, rest
# On importe la fonction unique et optimisée
from movement.navigation import optimised_search_cycle
from utils.logger import setup_logger
from vision.tool import (
    subscribe_with_target_res,
    camera_test_fast_color,
    camera_probe
)

from vision.camera import subscribe_camera, unsubscribe_camera, get_frame

# --- Configuration de base ---
default_ip = "172.16.1.163"   # Adresse NAO
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, "keras_model.h5")
LABELS_PATH = os.path.join(BASE_DIR, "labels.txt")

# --- Logger ---
logger = setup_logger()

# --- Stop manuelle (Ctrl+C) ---
stop_program = False
def signal_handler(sig, frame):
    global stop_program
    logger.warning("⛔ Arrêt manuel détecté (Ctrl+C)")
    stop_program = True
signal.signal(signal.SIGINT, signal_handler)


# --- Menu utilisateur ---
def display_menu():
    print("\n=== 🤖 MENU D'EXPLORATION NAO ===")
    print("1. Lancer le Cycle de Recherche Optimisé (Rotation + Grille)") # Nom mis à jour
    print("6. Dire un message")
    print("7. Mettre le robot assis")
    print("8. Test caméra (aperçu OpenCV rapide)")
    print("9. Diagnostic caméra (probe)")
    print("0. Quitter")
    return input("👉 Choisissez une option : ")


# --- Programme principal ---
def main(session):
    global stop_program

    logger.info("Initialisation des services...")

    # Initialisation des services NAO
    try:
        tts = session.service("ALTextToSpeech"); tts.setLanguage("French")
        motion_service = session.service("ALMotion")
        memory_service = session.service("ALMemory")
        video_service  = session.service("ALVideoDevice")
    except Exception as e:
        logger.error(f"Erreur d'initialisation des services NAOqi: {e}")
        return

    # Chargement du modèle IA
    try:
        model = load_model(MODEL_PATH, compile=False)
        with open(LABELS_PATH, "r") as f:
            class_names = [line.strip() for line in f.readlines()]
    except Exception as e:
        logger.error(f"Erreur chargement modèle ou labels: {e}")
        model = None
        class_names = []


    # Réveil du robot
    logger.info("Réveil du robot...")
    wake_up(session)

    # ✅ CORRECTION : Abonnement Caméra Frontale (0) avec YUV422 (9) et FPS réduit (10)
    cam_index = 1 # Caméra frontale pour la détection
    # color_space=9 (YUV422) et fps=10 pour la stabilité du réseau
    name_id = subscribe_camera(video_service, camera_index=cam_index, color_space=9, fps=10)
 
    if not name_id:
        logger.error("❌ Impossible d'activer la caméra ! Programme bloqué. Fin d'exécution.")
        rest(session)
        sys.exit(1)
    else:
        logger.info(f"✅ Caméra activée avec succès (index {cam_index})")


    while True:
        choice = display_menu()
        if stop_program:
            break

        if choice == "1":
            # Charger la carte JSON
            try:
                # Vérifiez que le dossier maps existe bien dans Recherche/maps/
                map_path = os.path.join(BASE_DIR, "maps/carte_03.json")
                if not os.path.exists(map_path):
                    logger.error(f"Fichier carte introuvable : {map_path}")
                    continue

                with open(map_path, "r") as f:
                     grid_map = np.array(json.load(f))
                
                logger.info(f"🗺️ Carte chargée. Lancement de la recherche avec détection...")
                
                # ✅ CORRECTION : Utiliser drive_robot_with_algo au lieu de run_algo_on_robot
                drive_robot_with_algo(
                    session, 
                    video_service, 
                    model, 
                    class_names, 
                    tts, 
                    name_id, 
                    grid_map
                )
                
            except Exception as e:
                logger.error(f"❌ Erreur lors de l'exécution de l'algo : {e}")
                
        elif choice == "0":
            logger.info("Arrêt demandé par l’utilisateur.")
            break

        elif choice == "6":
            tts.say("Bonjour, je suis le robot Nao.")
            continue

        elif choice == "7":
            rest(session)
            continue

        elif choice == "8":
            # Test caméra utilise l'index 1 par défaut, ou vous pouvez le fixer ici.
            camera_test_fast_color(video_service, cam_index=0) 

        elif choice == "9":
            camera_probe(video_service)
            
        else:
            print("⚙️ Option non reconnue ou non implémentée.")
            continue


        if stop_program:
            logger.warning("⛔ Arrêt manuel pendant l'exécution.")
            break

        print("\n=== ✅ Action terminée ===\n")
        time.sleep(0.5)

    # Nettoyage
    logger.info("Arrêt des services...")
    try:
        if name_id:
            unsubscribe_camera(video_service, name_id)
    except Exception:
        pass
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    rest(session)
    logger.info("Programme terminé proprement.")


# --- Lancement ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=default_ip)
    parser.add_argument("--port", type=int, default=9559)
    args = parser.parse_args()

    session = qi.Session()
    try:
        session.connect(f"tcp://{args.ip}:{args.port}")
    except RuntimeError:
        logger.error(f"Erreur de connexion à Naoqi sur {args.ip}:{args.port}")
        sys.exit(1)

    main(session)