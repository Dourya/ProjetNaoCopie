# movement/posture.py
# Fonctions pour gérer les postures du robot

import qi
import time

def wake_up(session):
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")

    # Vérifier si ALMotion est en pause ou en repos
    if not motion_service.robotIsWakeUp():
        print("Réveil du robot...")
        motion_service.wakeUp()
        time.sleep(1)  # attendre que le réveil se fasse correctement

    # Aller à la posture debout initiale
    posture_service.goToPosture("StandInit", 0.8)


def rest(session):
    motion_service = session.service("ALMotion")
    motion_service.rest()
