#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import signal
import argparse
import asyncio
import cv2
import numpy as np
import qi
from pyzbar import pyzbar

stop_program = False

# === ANGLES ENTRE QR CODES ===
ANGLES = {
    ("un", "deux"): -0.785,      # un → deux (-45°)
    ("deux", "trois"): 2.356,    # deux → trois (135°)
    ("trois", "un"): 2.356,      # trois → un (135°)

    ("deux", "un"): -2.356,       # deux → un (-145°)
    ("trois", "deux"): -2.356,   # trois → deux (-135°)
    ("un", "trois"): -2.356      # un → trois (-135°)
}

# === CTRL + C ===
def signal_handler(sig, frame):
    global stop_program
    stop_program = True
signal.signal(signal.SIGINT, signal_handler)

# === DETECTION FIABLE DES QR ===
def detect_qr(frame):
    # Agrandissement pour aider NAO (résolution faible)
    enlarged = cv2.resize(frame, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(enlarged, cv2.COLOR_BGR2GRAY)

    qrs = pyzbar.decode(gray)
    results = []
    for qr in qrs:
        try:
            txt = qr.data.decode("utf-8")
            results.append(txt)
        except:
            pass
    return results

# === CAMERA : AFFICHAGE EN DIRECT ===
async def show_camera_preview(video_service):
    subscriber = "CameraPreview"
    cam_id = 1  # Caméra du menton = meilleure pour QR
    capture = video_service.subscribeCamera(subscriber, cam_id, 1, 13, 20)
    cv2.namedWindow("Camera NAO", cv2.WINDOW_NORMAL)

    print("🎥 Affichage de la caméra NAO activé.")

    while not stop_program:
        img = video_service.getImageRemote(capture)

        if img:
            width = img[0]
            height = img[1]
            array = img[6]

            frame = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
            cv2.imshow("Camera NAO", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        await asyncio.sleep(0.01)

    video_service.unsubscribe(subscriber)
    cv2.destroyAllWindows()
    print("📷 Caméra fermée.")

# === AVANCER JUSQU'À DETECTER LE QR SUIVANT ===
async def scan_until(video_service, expected_qr):
    subscriber = "QRScanner"
    cam_id = 1
    capture = video_service.subscribeCamera(subscriber, cam_id, 1, 13, 20)

    print(f"🔍 Je cherche le QR : {expected_qr}")

    while not stop_program:
        img = video_service.getImageRemote(capture)
        if img:
            width = img[0]
            height = img[1]
            array = img[6]
            frame = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))

            found = detect_qr(frame)
            for qr in found:
                print(f"🔍 QR détecté à l'instant : {qr}")  # Message console à chaque QR détecté

            if expected_qr in found:
                print(f"📌 QR attendu trouvé : {expected_qr}")
                video_service.unsubscribe(subscriber)
                return True

        await asyncio.sleep(0.02)

    video_service.unsubscribe(subscriber)
    return False

# === PROGRAMME PRINCIPAL ===
async def main_async(session, route_list):
    motion = session.service("ALMotion")
    video = session.service("ALVideoDevice")
    tts = session.service("ALTextToSpeech")
    posture = session.service("ALRobotPosture")
    tts.setLanguage("French")

    # Réveil et posture
    tts.say("Je démarre la navigation.")
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.6)

    # 🔥 Baisser la tête dès le début
    tts.say("Je baisse la tête.")
    motion.setAngles("HeadPitch", 0.35, 0.15)
    time.sleep(1)

    # 🔥 Prévisualisation caméra en parallèle
    camera_task = asyncio.create_task(show_camera_preview(video))

    # ---- NAVIGATION ----
    for i in range(len(route_list) - 1):
        start = route_list[i]
        end = route_list[i+1]

        tts.say(f"Je vais de {start} à {end}")

        # 1. Angle à tourner
        if (start, end) not in ANGLES:
            tts.say("Je ne connais pas cet angle.")
            continue

        angle = ANGLES[(start, end)]

        # 2. Rotation
        tts.say("Je tourne.")
        motion.moveTo(0, 0, angle)

        # 3. Avance continue en scannant
        tts.say(f"Je cherche {end}")
        motion.move(0.15, 0, 0)  # vitesse lente sécurisée

        # 4. Attend la détection
        await scan_until(video, end)

        # 5. Stop
        motion.stopMove()
        tts.say(f"J'ai trouvé {end}")

    global stop_program
    stop_program = True

    tts.say("Parcours terminé.")
    motion.rest()
    await camera_task

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="172.16.1.163")
    parser.add_argument("--port", default=9559)
    parser.add_argument("--route", default="un,deux,trois,deux,un")

    args = parser.parse_args()
    route_list = [x.strip() for x in args.route.split(",")]

    session = qi.Session()
    session.connect(f"tcp://{args.ip}:{args.port}")

    asyncio.run(main_async(session, route_list))

if __name__ == "__main__":
    main()