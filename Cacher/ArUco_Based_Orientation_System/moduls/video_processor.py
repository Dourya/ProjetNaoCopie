import math
import cv2 as cv
import numpy as np
import threading
import queue
import time

class VideoProcessor:
    def __init__(self, camera_params_path='camera_parameters/intrinsic_iphone12.dat', source=0):
        self.source = source
        self.cmtx, self.dist = self.read_camera_parameters(camera_params_path)
        self.running = False
        self.thread = None
        
        # File d'attente pour communiquer avec l'interface graphique
        self.data_queue = queue.Queue(maxsize=1) 
        
        # Paramètres
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
        self.parameters = cv.aruco.DetectorParameters()
        self.marker_size = 7.8 # cm

    def read_camera_parameters(self, filepath):
        try:
            inf = open(filepath, 'r')
            cmtx = []
            dist = []

            #ignore first line
            line = inf.readline()
            for _ in range(3):
                line = inf.readline().split()
                line = [float(en) for en in line]
                cmtx.append(line)

            #ignore line that says "distortion"
            line = inf.readline()
            line = inf.readline().split()
            line = [float(en) for en in line]
            dist.append(line)
            
            inf.close()
            return np.array(cmtx), np.array(dist)
        except Exception as e:
            print(f"Erreur de lecture des paramètres caméra: {e}")
            return np.eye(3), np.zeros(5)

    def get_aruco_coords(self, points):
        # Définition des points 3D du marqueur
        obj_points = np.array([
            [0, 0, 0],
            [self.marker_size, 0, 0],
            [self.marker_size, self.marker_size, 0],
            [0, self.marker_size, 0]
        ], dtype='float32').reshape((4, 1, 3))

        ret, rvec, tvec = cv.solvePnP(obj_points, points, self.cmtx, self.dist)
        
        return rvec, tvec

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._process_loop)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def _process_loop(self):
        cap = cv.VideoCapture(self.source)

        # Gestion des fps
        fps = cap.get(cv.CAP_PROP_FPS)
        if fps <= 0: fps = 30 
        time_per_frame = 1 / fps 

        while self.running:
            start_time = time.time()

            ret, img = cap.read()
            if not ret:
                cap.set(cv.CAP_PROP_POS_FRAMES, 0)
                continue

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            #corners, ids, rejected = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            

            if hasattr(cv.aruco, "ArucoDetector"):
                detector = cv.aruco.ArucoDetector(self.aruco_dict, self.parameters)
                corners, ids, rejected = detector.detectMarkers(gray)
            else:
                corners, ids, rejected = cv.aruco.detectMarkers(
                    gray,
                    self.aruco_dict,
                    parameters=self.parameters
                )


            # 1. Liste pour stocker les données des marqueurs
            markers_data = []

            # Dessin des contours
            cv.aruco.drawDetectedMarkers(img, corners, ids)
            
            # 2. Boucle de récupération des données
            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    current_id = ids[i][0]
                    current_corners = corners[i]
                    
                    # Calcul des coordonnées
                    rvec, tvec = self.get_aruco_coords(current_corners)

                    # Calcul de l'orientation
                    # --- conversion du vecteur en matrice de rotation
                    rmat, _ = cv.Rodrigues(rvec)
                    # --- Extraction de l'angle dans la matrice de rotation
                    heading_x = rmat[0][0] 
                    heading_y = rmat[1][0]
                    # 3. Calcul de l'angle 2D (atan2 standard)
                    angle_rad = math.atan2(heading_y, heading_x)
                    angle_deg = math.degrees(angle_rad)
                    # --- Convertion de l'angle en degrés
                    angle_deg = math.degrees(angle_rad)
                    if angle_deg<-180:
                        angle_deg+=270
                    else:
                        angle_deg-=90
                    
                    # On ajoute un dictionnaire avec les infos dans la liste
                    markers_data.append({
                        "id": current_id,
                        "x": tvec[0][0],
                        "y": tvec[1][0],
                        "z": tvec[2][0],
                        "angle": angle_deg
                    })

            # Envoi à l'interface (Queue)
            if self.data_queue.full():
                try: self.data_queue.get_nowait()
                except queue.Empty: pass
            
            img_rgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
            
            # --- MODIFICATION MAJEURE ICI : On envoie la liste markers_data au lieu d'un texte ---
            self.data_queue.put((img_rgb, markers_data))
            
            # Pause intelligente
            processing_time = time.time() - start_time
            wait_time = time_per_frame - processing_time
            if wait_time > 0:
                time.sleep(wait_time)

        cap.release()