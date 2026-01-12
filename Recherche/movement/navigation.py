# movement/navigation.py

import time
import math
import cv2
import numpy as np # Nécessaire pour la vérification de luminosité
from vision.camera import get_frame
from vision.detection import detect_nao

# ==============================================================
# === GÉNÉRATEURS DE MOUVEMENT =================================
# ==============================================================


from fonction_score import Seeker, Explorer
from vision.camera import get_frame
from vision.detection import detect_nao

def drive_robot_with_algo(session, video_service, model, class_names, tts, name_id, grid_map, start_pos=(60, 10)):
    """
    Pilote le robot en utilisant la logique de l'Explorer (A*).
    """
    motion_service = session.service("ALMotion")
    memory_service = session.service("ALMemory")
    
    # --- 1. CORRECTION ECHELLE ---
    # Si la map est en cm (350x710 pour 3.5m x 7.1m), alors 1 case = 0.01m
    CELL_SIZE = 0.01  
    
    # --- 2. CORRECTION POSITION ---
    # On utilise bien l'argument start_pos et on supprime l'écrasement
    start_x, start_y = start_pos 
    print(f"📍 Démarrage Algo aux coordonnées virtuelles : {start_x}, {start_y}")
    
    # Vérification si on commence dans un mur
    try:
        if grid_map[start_y][start_x] == 1:
            print("⚠️ ATTENTION : La position de départ est DANS UN MUR virtuel !")
            tts.say("Je suis dans un mur !")
            return
    except IndexError:
        print("⚠️ ATTENTION : Position de départ hors limite !")
        return

    seeker = Seeker(
        pos=(start_x, start_y),
        vision_angle=70,
        max_distance=300, 
        direction_angle=0,
        grid=grid_map,
        target=[], 
        size=10 
    )
    
    explorer = Explorer(seeker)
    current_path = []
    
    print("🚀 Navigation active. J'attends les ordres...")
    motion_service.wakeUp()
    
    while True:
        # A. Vision
        frame = get_frame(video_service, name_id)
        if frame is not None:
            detected, class_name, conf = detect_nao(frame, model, class_names)
            
            # Seuil de confiance augmenté à 0.98 pour éviter les faux positifs
            if detected and conf > 0.98:
                print(f"🛑 STOP : Cible détectée ({class_name} à {conf:.2f})")
                tts.say(f"J'ai trouvé {class_name}")
                motion_service.stopMove()
                break 

        # B. Mise à jour carte mentale
        explorer.mettre_a_jour_vue()
        
        # C. Décision
        if not current_path:
            target_pos = explorer.trouver_cible_lointaine()
            
            if target_pos is None:
                print("🏁 FIN : Plus aucune zone inconnue accessible.")
                tts.say("Exploration terminée.")
                break
                
            path = explorer.trouver_chemin_astar((seeker.x, seeker.y), target_pos)
            
            if path:
                current_path = path
                print(f"🛣️ Nouveau chemin calculé ({len(path)} pas) vers {target_pos}")
            else:
                print("⚠️ Bloqué (pas de chemin A*). Rotation de secours.")
                motion_service.moveTo(0, 0, math.radians(45))
                seeker.tourner(45)
                continue

        # D. Mouvement
        if current_path:
            next_x, next_y = current_path.pop(0)
            
            dx = next_x - seeker.x
            dy = next_y - seeker.y
            
            if dx == 0 and dy == 0: continue
                
            dist_meters = math.sqrt(dx**2 + dy**2) * CELL_SIZE
            
            target_angle_rad = math.atan2(dy, dx)
            current_angle_rad = math.radians(seeker.direction_angle)
            rotation_needed = target_angle_rad - current_angle_rad
            rotation_needed = (rotation_needed + math.pi) % (2 * math.pi) - math.pi
            
            # Exécution
            # On ne tourne que si l'angle est significatif (> 10 degrés)
            if abs(rotation_needed) > 0.17: 
                motion_service.moveTo(0, 0, rotation_needed)
            
            if dist_meters > 0:
                motion_service.moveTo(dist_meters, 0, 0)
            
            # Mise à jour virtuelle
            seeker.x = next_x
            seeker.y = next_y
            seeker.direction_angle = round(math.degrees(math.atan2(dy, dx)) / 45) * 45 % 360
            
    motion_service.rest()

def rotate_on_place_generator(motion_service, num_steps=8, step_deg=45, pause=0.3):
    """Effectue une rotation complète sur place."""
    step_rad = math.radians(step_deg)
    for i in range(num_steps):
        motion_service.moveTo(0.0, 0.0, step_rad)
        time.sleep(pause)
        yield True

def cautious_forward_generator(motion_service, memory_service, num_steps=5, step=0.3, threshold=0.4, pause=0.2):
    """Avance prudemment un certain nombre de pas."""
    for _ in range(num_steps):
        motion_service.moveTo(step, 0.0, 0.0)
        time.sleep(pause)
        yield True
    motion_service.stopMove()


def grid_exploration_generator(motion_service, num_lines=3, line_length=0.7, lateral_step=0.2, pause=0.5):
    """
    Explore l'environnement selon un motif de grille systématique (Boustrophedon).
    """
    print("Exploration: Grille Systématique (Couverture complète)")
    turn_angle = math.pi / 2 
    
    for i in range(num_lines):
        # 1. Avancer le long de la ligne
        print(f"Ligne {i+1}: Avance de {line_length}m")
        motion_service.moveTo(line_length, 0.0, 0.0)
        time.sleep(pause)
        yield True 

        # --- Détermination du prochain virage pour passer à la ligne suivante ---
        if i < num_lines - 1:
            # 2. Tourner 90° (préparation au pas latéral)
            angle = -turn_angle if i % 2 == 0 else turn_angle
            motion_service.moveTo(0.0, 0.0, angle)
            time.sleep(pause)
            yield True

            # 3. Avance latérale courte
            print(f"Ligne {i+1}: Pas latéral de {lateral_step}m")
            motion_service.moveTo(lateral_step, 0.0, 0.0) 
            time.sleep(pause)
            yield True

            # 4. Tourner 90° pour se remettre dans l'axe de la ligne suivante
            motion_service.moveTo(0.0, 0.0, angle)
            time.sleep(pause)
            yield True

    motion_service.stopMove()
    print("Exploration en grille terminée.")


# ==============================================================
# === FONCTION PRINCIPALE OPTIMISÉE ============================
# ==============================================================

def optimised_search_cycle(session, video_service, model, class_names, tts, name_id):
    """
    Exécute un cycle de recherche optimal : rotation visuelle, puis exploration en grille.
    """
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    sonar_service = session.service("ALSonar")
    memory_service = session.service("ALMemory")

    # Configuration de la sécurité et de la posture
    sonar_service.subscribe("ObstacleDetector")
    time.sleep(0.5)
    motion_service.wakeUp()
    posture_service.goToPosture("StandInit", 1.0)

    # Stratégie : Rotation (Balayage rapide) puis Grille (Couverture complète)
    exploration_phases = [
        ("Rotation Balayage", rotate_on_place_generator(motion_service, num_steps=8)),
        ("Grille Systématique", grid_exploration_generator(motion_service, num_lines=3, line_length=0.7)) 
    ]

    detected_object = None
    
    # Itération à travers les phases
    for phase_name, explorer in exploration_phases:
        print(f"\n--- 🧭 Phase : {phase_name} ---")

        for _ in explorer: # Le générateur met le corps en mouvement
            
            # --- Balayage de Tête + Détection ---
            for head_yaw in [-0.5, 0.0, 0.5]: 
                
                # 🛡️ GESTION DE LA DÉCONNEXION lors du mouvement de tête
                try:
                    motion_service.setAngles("HeadYaw", head_yaw, 0.2)
                    time.sleep(0.3) 
                except RuntimeError as e:
                    print(f"[{time.strftime('%H:%M:%S')}] 🚨 Connexion perdue lors du mouvement : {e}. Sortie forcée.")
                    detected_object = True 
                    break 

                # Capture caméra
                frame = get_frame(video_service, name_id)
                if frame is None:
                    continue

                # 💡 VÉRIFICATION ANTI-ÉCRAN NOIR : Si l'image est trop sombre, la sauter
                mean_brightness = frame.mean() 
                if mean_brightness < 10.0:
                    # print(f"[{time.strftime('%H:%M:%S')}] 🚧 Image trop sombre (moyenne: {mean_brightness:.1f}), sautée.")
                    continue
                
                # 🔍 Détection visuelle (Nao cible et QRCode)
                detected, class_name, confidence = detect_nao(frame, model, class_names, threshold=0.95)

                # ✅ AFFICHAGE EN PERMANENCE
                label = f"{class_name} - {confidence*100:.1f}%"
                print(f"🔍 IA → Classe: {class_name} | Confiance: {confidence:.3f}")

                color = (0, 255, 0) if detected else (0, 180, 255)
                cv2.putText(frame, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
                cv2.imshow("📷 Vue NAO (détection)", frame)

                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    print("⛔ Arrêt manuel via la fenêtre vidéo.")
                    motion_service.stopMove()
                    detected_object = True 
                    break

                # ✅ Si NAO détecté : arrêt immédiat
                if detected:
                    print(f"✅ NAO détecté : {label}")
                    tts.say(f"J'ai trouvé un {class_name}")
                    detected_object = class_name
                    motion_service.stopMove()
                    break 

            if detected_object:
                break 
            
            # --- Gestion des obstacles (Sonar) ---
            left = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            right = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")
            if min(left, right) < 0.40:
                print("⚠️ Obstacle détecté, rotation d'évitement.")
                # 🛡️ GESTION DE LA DÉCONNEXION lors de l'évitement
                try:
                    motion_service.stopMove()
                    if left > right:
                        motion_service.moveTo(0, 0, 1.57) # Tourne à gauche
                    else:
                        motion_service.moveTo(0, 0, -1.57) # Tourne à droite
                    time.sleep(1.0) 
                except RuntimeError as e:
                    print(f"[{time.strftime('%H:%M:%S')}] 🚨 Connexion perdue lors de l'évitement : {e}. Arrêt.")
                    detected_object = True
                    break
        
        if detected_object:
            break
    
    # --- Nettoyage Final ---
    motion_service.setAngles("HeadYaw", 0.0, 0.2)
    sonar_service.unsubscribe("ObstacleDetector")
    cv2.destroyAllWindows()
    motion_service.rest()
    print("✅ Cycle de recherche terminé proprement.")

    return detected_object is not None