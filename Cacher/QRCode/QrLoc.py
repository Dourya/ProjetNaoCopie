#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import sys
import signal
import qi
import argparse

# ================================
# ⚡ Gestion arrêt propre
# ================================
stop_program = False

def signal_handler(sig, frame):
    global stop_program
    stop_program = True

signal.signal(signal.SIGINT, signal_handler)

# ================================
# 📌 Matrice intrinsèque NAO (approx)
# ================================
CMTX = np.array([
    [570.0, 0.0, 320.0],
    [0.0, 570.0, 240.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

DIST = np.zeros((5, 1))  # NAO = pas de distorsion notable
QR_SIZE = 0.10  # taille QR en mètres

# ================================
# 🔍 Détection QR + solvePnP
# ================================
def get_qr_pose(frame):
    qr = cv2.QRCodeDetector()
    ret, points = qr.detect(frame)

    if not ret or points is None:
        return None

    pts = points[0].reshape((4, 2)).astype(np.float32)

    # Référence QR dans le plan 3D
    qr_edges = np.array([
        [0, 0, 0],
        [0, QR_SIZE, 0],
        [QR_SIZE, QR_SIZE, 0],
        [QR_SIZE, 0, 0]
    ], dtype=np.float32).reshape((4,1,3))

    ret2, rvec, tvec = cv2.solvePnP(qr_edges, pts, CMTX, DIST)

    if not ret2:
        return None

    return pts, rvec, tvec

# ================================
# 🎥 Flux caméra NAO
# ================================
def show_camera(session):
    video = session.service("ALVideoDevice")

    name = "qrStream"
    cam = 1  # caméra basse
    resolution = 1  # 320x240
    color = 13  # RGB
    fps = 20

    handle = video.subscribeCamera(name, cam, resolution, color, fps)
    cv2.namedWindow("Camera NAO", cv2.WINDOW_NORMAL)

    while not stop_program:
        img = video.getImageRemote(handle)
        if img is None:
            continue

        w, h = img[0], img[1]
        frame = np.frombuffer(img[6], dtype=np.uint8).reshape((h, w, 3))

        # ----- Détection QR -----
        result = get_qr_pose(frame)
        if result is not None:
            pts, rvec, tvec = result
            X, Y, Z = map(float, tvec)

            # Affichage coordonnées
            cv2.putText(frame, f"X = {X:.2f} m", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(frame, f"Y = {Y:.2f} m", (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            cv2.putText(frame, f"Z = {Z:.2f} m", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

            # Dessine contour QR
            pts = pts.astype(int)
            for i in range(4):
                p1 = tuple(pts[i])
                p2 = tuple(pts[(i+1)%4])
                cv2.line(frame, p1, p2, (255,0,0), 2)

        # Affichage flux
        cv2.imshow("Camera NAO", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video.unsubscribe(handle)
    cv2.destroyAllWindows()

# ================================
# 🔧 MAIN
# ================================
def main():
    parser = argparse.ArgumentParser(description="Flux caméra NAO + détection QR code")
    parser.add_argument("--ip", type=str, default="172.16.1.164", help="IP du robot NAO")
    parser.add_argument("--port", type=int, default=9559, help="Port NAOqi")
    args = parser.parse_args()

    # Connexion NAOqi
    session = qi.Session()
    try:
        session.connect(f"tcp://{args.ip}:{args.port}")
        print(f"✅ Connecté à NAOqi ({args.ip}:{args.port})")
    except RuntimeError:
        print("❌ Impossible de se connecter à NAOqi")
        sys.exit(1)

    # Lancer flux caméra
    print("🔵 Flux caméra lancé. Appuyez sur Q pour quitter.")
    show_camera(session)

if __name__ == "__main__":
    main()