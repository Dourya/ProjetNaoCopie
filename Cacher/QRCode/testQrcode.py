#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import signal
import argparse
import asyncio
import concurrent.futures
import cv2
import numpy as np
import qi
from pyzbar import pyzbar

# === CONFIGURATION ===
WIDTH, HEIGHT = 160, 120
FPS = 20
stop_program = False
capture_id = None


# === GESTION SIGNAUX ===
def signal_handler(sig, frame):
    global stop_program
    stop_program = Truea(subscriber_name, camera_id, 0, 11, FPS)
    print("\n⛔ Arrêt manuel (Ctrl+C)")

signal.signal(signal.SIGINT, signal_handler)


# === INSTRUCTIONS ===
def execute_instruction(motion_service, tts, instruction):
    instruction = instruction.lower().strip()
    if instruction == "un":
        tts.say("Je tourne à droite")
        motion_service.moveTo(0, 0, -1.57)

    elif instruction == "deux":
        tts.say("Je tourne à gauche")
        motion_service.moveTo(0, 0, 1.57)
        
    elif instruction == "trois":
        tts.say("Je m'arrête")
        motion_service.stopMove()
    elif instruction == " P ":
        tts.say("je m'appelle nao ,je me cache")
    else:
        tts.say(f"Instruction inconnue : {instruction}")


# === CAPTURE & DETECTION ===
async def camera_loop(video_service, motion_service, tts, camera_id, subscriber_name):
    global stop_program, capture_id
    window_name = "Caméra du menton - QR Code"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    capture_id = video_service.subscribeCamera(subscriber_name, camera_id, 0, 11, FPS)
    print(f"🎥 Caméra du menton lancée : {WIDTH}x{HEIGHT} @ {FPS} FPS")

    last_detection = None
    detected_recently = set()

    executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

    while not stop_program:
        img = video_service.getImageRemote(capture_id)
        if img is None:
            await asyncio.sleep(0.01)
            continue

        w, h = img[0], img[1]
        array = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 3))
        frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

        loop = asyncio.get_event_loop()
        qrcodes = await loop.run_in_executor(executor, detect_qr, frame)

        for qr_text in qrcodes:
            if qr_text not in detected_recently:
                print(f"🔍 QR Code détecté : {qr_text}")
                asyncio.create_task(run_instruction_async(motion_service, tts, qr_text))
                detected_recently.add(qr_text)

        if last_detection is None or (time.time() - last_detection) > 3:
            detected_recently.clear()
            last_detection = time.time()

        cv2.imshow(window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_program = True
            break

    cv2.destroyAllWindows()
    video_service.unsubscribe(subscriber_name)
    print("✅ Caméra arrêtée proprement")


# === DETECTION QR ===
def detect_qr(frame):
    resized = cv2.resize(frame, (320, 320), interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    qrcodes = pyzbar.decode(gray)
    return [qr.data.decode("utf-8") for qr in qrcodes]


# === EXECUTION ASYNCHRONE DES INSTRUCTIONS ===
async def run_instruction_async(motion_service, tts, instruction):
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, execute_instruction, motion_service, tts, instruction)


# === MAIN ===
async def main_async(session):
    video_service = session.service("ALVideoDevice")
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tts = session.service("ALTextToSpeech")
    tts.setLanguage("French")

    print("🤖 Réveil du robot...")
    motion_service.wakeUp()
    posture_service.goToPosture("StandInit", 0.8)

    # 🆕 AJOUT : le robot baisse la tête avant d'avancer
    tts.say("Je baisse la tête pour mieux voir")
    motion_service.setAngles("HeadPitch", 0.4, 0.2)
    time.sleep(1)

    # 🚀 AVANCE AUTOMATIQUE APRÈS AVOIR BAISSÉ LA TÊTE
    tts.say("J'avance")
    motion_service.moveTo(0.3, 0, 0)

    camera_id = 1
    subscriber_name = "CameraMenton"

    await camera_loop(video_service, motion_service, tts, camera_id, subscriber_name)

    print("🛑 Fin du programme, mise au repos du robot")
    motion_service.rest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="172.16.1.164", help="IP du robot NAO/Pepper")
    parser.add_argument("--port", type=int, default=9559, help="Port NAOqi")
    args = parser.parse_args()

    session = qi.Session()
    try:
        session.connect(f"tcp://{args.ip}:{args.port}")
        print(f"✅ Connecté à NAOqi ({args.ip}:{args.port})")
    except RuntimeError:
        print("❌ Erreur de connexion à NAOqi")
        sys.exit(1)

    asyncio.run(main_async(session))


if __name__ == "__main__":
    main()