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

# =====================
# 📌 Angles
# =====================
ANGLES = {
    ("un", "deux"): -0.785,      # un → deux (-45°)
    ("deux", "trois"): 2.356,    # deux → trois (135°)
    ("trois", "un"): -2.356,      # trois → un (135°)

    ("deux", "un"): -2.356,       # deux → un (-145°)
    ("trois", "deux"): -2.356,   # trois → deux (-135°)
    ("un", "trois"): -2.356      # un → trois (-135°)
}

# =====================
# CTRL+C
# =====================
def signal_handler(sig, frame):
    global stop_program
    stop_program = True
signal.signal(signal.SIGINT, signal_handler)


# =====================
# 🔍 Optimisation image
# =====================
def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    gray = cv2.bilateralFilter(gray, 5, 55, 55)
    return gray


def detect_qr(frame):
    processed = preprocess(frame)
    codes = pyzbar.decode(processed)
    result = []
    for c in codes:
        try:
            result.append(c.data.decode("utf-8"))
        except:
            pass
    return result


# =====================
#  Preview caméra 
# =====================
async def show_camera(video):
    sub = "Preview"
    cap = video.subscribeCamera(sub, 1, 1, 13, 20)
    cv2.namedWindow("Camera NAO", cv2.WINDOW_NORMAL)

    while not stop_program:
        img = video.getImageRemote(cap)
        if img:
            w, h = img[0], img[1]
            frame = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 3))
            cv2.imshow("Camera NAO", frame)
        if cv2.waitKey(1) == ord('q'):
            break
        await asyncio.sleep(0.01)

    video.unsubscribe(cap)
    cv2.destroyAllWindows()


# =====================
# 🔍 Scan en avançant (bloquant)
# =====================
async def scan_and_wait(video, expected_qr):
    sub = "Scanner"
    cap = video.subscribeCamera(sub, 1, 1, 13, 20)

    print(f"🔍 Je cherche le QR : {expected_qr}")

    while not stop_program:
        img = video.getImageRemote(cap)
        if img:
            w, h = img[0], img[1]
            frame = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 3))

            found = detect_qr(frame)

            if len(found) > 0:
                print(f"QR détectés : {found}")

            if expected_qr in found:
                print(f"✔ QR attendu trouvé : {expected_qr}")
                video.unsubscribe(cap)
                return True

        await asyncio.sleep(0.02)

    video.unsubscribe(cap)
    return False


# =====================
# ⭐ PROGRAMME PRINCIPAL ⭐
# =====================
async def main_async(session, route):
    motion = session.service("ALMotion")
    video = session.service("ALVideoDevice")
    tts = session.service("ALTextToSpeech")
    posture = session.service("ALRobotPosture")
    tts.setLanguage("French")

    # Réveil
    tts.say("Je commence le parcours.")
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.6)

    # Tête vers le bas pour QR
    motion.setAngles("HeadPitch", 0.35, 0.2)
    time.sleep(1)

    # 🎥 caméra active jusqu’à la fin
    camera_task = asyncio.create_task(show_camera(video))

    # ======== NAVIGATION ==========
    for i in range(len(route) - 1):
        start = route[i]
        end = route[i + 1]

        tts.say(f"Je vais de {start} vers {end}.")

        if (start, end) not in ANGLES:
            tts.say("Je ne connais pas cet angle.")
            continue

        angle = ANGLES[(start, end)]

        # Rotation
        motion.moveTo(0, 0, angle)

        # Avance lente
        motion.move(0.10, 0, 0)

        # Attend le bon QR
        await scan_and_wait(video, end)

        # Stop
        motion.stopMove()
        tts.say(f"J'ai trouvé {end}.")

    # Fin
    global stop_program
    stop_program = True

    tts.say("Parcours terminé.")
    motion.rest()

    await camera_task


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="172.16.1.164")
    parser.add_argument("--port", default=9559)
    parser.add_argument("--route", default="un,deux,trois,deux,un")
    args = parser.parse_args()

    route = [x.strip() for x in args.route.split(",")]

    session = qi.Session()
    session.connect(f"tcp://{args.ip}:{args.port}")

    asyncio.run(main_async(session, route))

if __name__ == "__main__":
    main()