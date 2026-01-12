import cv2
import numpy as np

# --- CONFIGURATION ---
VIDEO_FILE = 'calibration/calibrate_CCRI.mp4'  # Remplacez par le nom de votre vidéo
OUTPUT_FILE = 'camera_parameters/intrinsic_CCRI.dat'

# Dimensions du damier (nombre de coins INTERNES, pas les cases)
# Pour le damier standard OpenCV : 9 colonnes internes, 6 lignes internes
CHECKERBOARD = (9, 6) 
# ---------------------

def calibrate():
    # Critères d'arrêt pour l'affinement des coins (précision)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Préparer les points 3D théoriques du damier (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

    objpoints = [] # Points 3D dans le monde réel
    imgpoints = [] # Points 2D dans l'image

    cap = cv2.VideoCapture(VIDEO_FILE)
    if not cap.isOpened():
        print(f"Erreur: Impossible d'ouvrir la vidéo {VIDEO_FILE}")
        return

    count = 0
    success_count = 0
    print("Analyse de la vidéo en cours...")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # On ne traite qu'une image sur 10 pour aller plus vite et éviter les doublons
        if count % 10 == 0:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Trouver les coins du damier
            ret_chess, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

            if ret_chess:
                objpoints.append(objp)
                # Affiner la précision des coins
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)
                success_count += 1
                
                # Dessiner pour vérifier (optionnel, ralentit)
                # cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret_chess)
                # cv2.imshow('Calibration', frame)
                # cv2.waitKey(1)
                print(f"Image {success_count} capturée", end='\r')

        count += 1

    cap.release()
    cv2.destroyAllWindows()
    print(f"\nTerminé. {success_count} images valides utilisées.")

    if success_count < 10:
        print("Erreur: Pas assez d'images détectées pour une bonne calibration.")
        return

    # --- CALIBRATION ---
    print("Calcul des paramètres intrinsèques... (patience)")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # --- SAUVEGARDE AU FORMAT DU PROJET ---
    print(f"Sauvegarde dans {OUTPUT_FILE}...")
    
    with open(OUTPUT_FILE, 'w') as f:
        f.write("intrinsic:\n")
        # Écriture de la matrice 3x3
        for row in mtx:
            f.write(f"{row[0]} {row[1]} {row[2]} \n")
        
        f.write("distortion:\n")
        # Écriture des coefficients de distorsion (applatis sur une ligne)
        dist_flat = dist.ravel()
        f.write(f"{dist_flat[0]} {dist_flat[1]} {dist_flat[2]} {dist_flat[3]} {dist_flat[4]}\n")

    print("Succès ! Vous pouvez maintenant utiliser ce fichier.")
    print("Matrice Caméra :\n", mtx)
    print("Distorsion :\n", dist)

if __name__ == '__main__':
    calibrate()