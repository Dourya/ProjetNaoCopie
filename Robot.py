#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import os
import sys
import time
import signal
import argparse
import threading
import cv2
import numpy as np
import qi


# ============================================================
#  Classe Robot
# ============================================================

class Robot:
    def __init__(self, ip, port, carte=None):
        self.ip = ip
        self.port = port
        self.map = carte
        self.stop_program = False

        # Connexion NAOqi
        self.session = self.connect_to_robot(ip, port)

        # Services NAOqi
        self.motion = self.session.service("ALMotion")
        self.tts = self.session.service("ALTextToSpeech")
        self.memory = self.session.service("ALMemory")
        self.video = self.session.service("ALVideoDevice")
        self.tts.setLanguage("French")

        # Threads
        self.cam_thread = None
        self.cam_stop_event = threading.Event()

        self.motion_thread = None
        self.motion_stop_event = threading.Event()

        # Capture image actuelle
        self.image_lock = threading.Lock()
        self.current_image = None

        # Gestion CTRL+C
        signal.signal(signal.SIGINT, self.signal_handler)

    # --------------------------------------------------------
    # Connexion
    # --------------------------------------------------------
    def connect_to_robot(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
            print(f"🤖 Connecté au robot {ip}:{port}")
        except RuntimeError:
            print("❌ Impossible de se connecter au robot.")
            sys.exit(1)
        return session

    # --------------------------------------------------------
    # Gestion SIGINT
    # --------------------------------------------------------
    def signal_handler(self, sig, frame):
        print("⛔ Arrêt manuel (Ctrl+C)")
        self.stop_program = True

    # --------------------------------------------------------
    # Menu
    # --------------------------------------------------------
    def show_menu(self):
        print("\n=== 🤖 MENU NAO (OOP) ===")
        print("1. Réveiller")
        print("2. Repos")
        print("3. Dire un message")
        print("4. Démarrer caméra")
        print("5. Avancer jusqu’à obstacle")
        print("6. Avancer + caméra")
        print("0. Quitter")
        return input("👉 Choix : ")

    # --------------------------------------------------------
    # Fonctions bas niveau
    # --------------------------------------------------------
    def wake_up(self):
        self.motion.wakeUp()

    def rest(self):
        self.motion.rest()

    def speak(self, msg):
        self.tts.say(msg)

    # ============================================================
    # THREAD CAMERA
    # ============================================================
    def camera_thread_func(self):
        name = f"cam_thread_{int(time.time())}"

        try:
            self.video.unsubscribe(name)
        except:
            pass

        # Subscribe caméra Naoqi
        try:
            cam_id = self.video.subscribeCamera(name, 1, 0, 9, 10)
        except Exception as e:
            print(f"❌ Impossible d'accéder à la caméra : {e}")
            return

        cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)
        print("🎥 Caméra démarrée — appuyer sur Q pour arrêter")

        while not self.cam_stop_event.is_set():
            img = self.video.getImageRemote(cam_id)
            if img is None:
                continue
            # self.show_image_details(img)
            w, h = img[0], img[1]
            
            # arr = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 3))
            yuv = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 2))
            arr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)

            with self.image_lock:
                self.current_image = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            cv2.imshow("Camera", self.current_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cam_stop_event.set()
                break

        self.video.unsubscribe(name)
        cv2.destroyWindow("Camera")
        print("📷 Thread caméra arrêté proprement.")

    def show_image_details(self,img):
        print("Width :",img[0])
        print("Height :",img[1])
        print("Number of layers :",img[2])
        print("ColorSpace :",img[3])

    def start_camera_thread(self):
        if self.cam_thread is None or not self.cam_thread.is_alive():
            self.cam_stop_event.clear()
            self.cam_thread = threading.Thread(target=self.camera_thread_func)
            self.cam_thread.start()
        else:
            print("⚠️ Caméra déjà active.")

    # ============================================================
    # THREAD MOUVEMENT
    # ============================================================
    def motion_thread_func(self):
        print("🚶 Démarrage du mouvement…")
        self.motion.wakeUp()

        posture = self.session.service("ALRobotPosture")
        try:
            posture.goToPosture("StandInit", 0.6)
        except:
            pass

        self.motion.moveInit()
        self.motion.moveToward(0.2, 0, 0)

        while not self.motion_stop_event.is_set():
            bump_l = self.memory.getData("LeftBumperPressed")
            bump_r = self.memory.getData("RightBumperPressed")
            if bump_l or bump_r:
                print("🟥 Bumper touché → arrêt")
                break

            time.sleep(0.05)

        self.motion.stopMove()
        print("🏁 Mouvement terminé.")

    def start_motion_thread(self):
        if self.motion_thread is None or not self.motion_thread.is_alive():
            self.motion_stop_event.clear()
            self.motion_thread = threading.Thread(target=self.motion_thread_func)
            self.motion_thread.start()
        else:
            print("⚠️ Mouvement déjà en cours.")

    # ============================================================
    # MODE COMBINÉ : CAMERA + MOUVEMENT
    # ============================================================
    def start_motion_with_camera(self):
        if (self.cam_thread and self.cam_thread.is_alive()) or \
           (self.motion_thread and self.motion_thread.is_alive()):
            print("⚠️ Caméra ou mouvement déjà actif.")
            return

        print("🤖 Départ : caméra + mouvement")

        self.cam_stop_event.clear()
        self.motion_stop_event.clear()

        self.cam_thread = threading.Thread(target=self.camera_thread_func)
        self.motion_thread = threading.Thread(target=self.motion_thread_func)

        self.cam_thread.start()
        self.motion_thread.start()

        while not (self.cam_stop_event.is_set() or self.motion_stop_event.is_set()):
            time.sleep(0.1)

        self.cam_stop_event.set()
        self.motion_stop_event.set()

        print("🛑 Fin du mode combiné.")

    # ============================================================
    # BOUCLE PRINCIPALE
    # ============================================================
    def main_loop(self):
        while not self.stop_program:
            choice = self.show_menu()

            if choice == "1": self.wake_up()
            elif choice == "2": self.rest()
            elif choice == "3": self.speak(input("Message : "))
            elif choice == "4": self.start_camera_thread()
            elif choice == "5": self.start_motion_thread()
            elif choice == "6": self.start_motion_with_camera()
            elif choice == "0":
                print("👋 Fin du programme.")
                self.stop_program = True
            else:
                print("❌ Choix incorrect.")

            time.sleep(0.3)

        # Arrêt propre
        self.cam_stop_event.set()
        self.motion_stop_event.set()
        if self.cam_thread: self.cam_thread.join()
        if self.motion_thread: self.motion_thread.join()

        self.rest()
        print("🟢 Programme terminé proprement.")


# ============================================================
# Point d’entrée
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="172.16.1.164")
    parser.add_argument("--port", type=int, default=9559)
    args = parser.parse_args()

    robot = Robot(args.ip, args.port)
    robot.main_loop()
