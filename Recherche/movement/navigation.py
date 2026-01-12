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


def run_algo_on_robot(session, grid_map):
    motion = session.service("ALMotion")
    
    # 1. Configurer le robot virtuel (Seeker) au même endroit que le vrai robot
    # Attention : x, y sont des indices de case (ex: 60, 10)
    seeker = Seeker(pos=(60, 10), vision_angle=70, max_distance=300, 
                    direction_angle=0, grid=grid_map, target=[], size=10)
    
    explorer = Explorer(seeker)
    current_path = []
    METERS_PER_CELL = 0.04  # À ajuster : 1 case = 4 cm ?

    print("🚀 Démarrage Algo A* sur le vrai robot...")
    motion.wakeUp()

    while True:
        # --- A. Partie Algorithme (Cerveau) ---
        explorer.mettre_a_jour_vue()
        
        # Si pas de chemin, on cherche une cible
        if not current_path:
            target = explorer.trouver_cible_lointaine()
            if target is None:
                print("Exploration finie.")
                break
            
            path = explorer.trouver_chemin_astar((seeker.x, seeker.y), target)
            if path:
                current_path = path
            else:
                # Blocage virtuel -> Rotation réelle et virtuelle
                print("Rotation de déblocage")
                seeker.tourner(45)
                motion.moveTo(0, 0, math.radians(45))
                continue

        # --- B. Partie Mouvement (Jambes) ---
        if current_path:
            next_x, next_y = current_path.pop(0)
            
            # Calcul du mouvement réel
            dx = next_x - seeker.x
            dy = next_y - seeker.y
            dist_m = math.sqrt(dx**2 + dy**2) * METERS_PER_CELL
            
            # Calcul de l'angle
            angle_target = math.atan2(dy, dx)
            angle_robot = math.radians(seeker.direction_angle)
            rotation = angle_target - angle_robot
            rotation = (rotation + math.pi) % (2 * math.pi) - math.pi # Normalisation

            # Exécution physique
            if abs(rotation) > 0.1:
                motion.moveTo(0, 0, rotation)
            motion.moveTo(dist_m, 0, 0)
            
            # Mise à jour virtuelle
            seeker.x, seeker.y = next_x, next_y
            seeker.direction_angle = math.degrees(angle_target) % 360

def drive_robot_with_algo(session, video_service, model, class_names, tts, name_id, grid_map):
    """
    Pilote le robot en utilisant la logique de l'Explorer (A*).
    """
    motion_service = session.service("ALMotion")
    memory_service = session.service("ALMemory")
    
    # --- CONFIGURATION ---
    # Échelle : Combien de mètres mesure une case de votre grille ?
    # Exemple : Si votre carte fait 3.5m de large et la grille 350 pixels, SCALE = 0.01 (1cm)
    CELL_SIZE = 0.04  # 4 cm par case (à ajuster selon la taille réelle de votre arène)
    
    # Initialisation du robot virtuel (Seeker)
    # Attention : Il faut que la position de départ (start_x, start_y) corresponde à la réalité !
    start_x, start_y = 60, 10  # Exemple arbitraire, à calibrer
    
    seeker = Seeker(
        pos=(start_x, start_y),
        vision_angle=70,
        max_distance=300, # En cases
        direction_angle=0,
        grid=grid_map,
        target=[], # On ne connait pas la cible réelle, c'est ce qu'on cherche
        size=10 # Taille du robot en cases
    )
    
    explorer = Explorer(seeker)
    current_path = []
    
    print("🚀 Démarrage de la navigation algorithmique...")
    motion_service.wakeUp()
    
    # Boucle principale (similaire à recherche_affichage mais physique)
    while True:
        # 1. Vérification Visuelle (Est-ce que je vois l'autre robot ?)
        frame = get_frame(video_service, name_id)
        if frame is not None:
            detected, class_name, conf = detect_nao(frame, model, class_names)
            if detected:
                print(f"✅ CIBLE TROUVÉE : {class_name} ({conf:.2f})")
                tts.say(f"J'ai trouvé {class_name} !")
                motion_service.stopMove()
                break # Fin du jeu
        
        # 2. Mise à jour de la carte mentale (Virtuel)
        explorer.mettre_a_jour_vue()
        
        # 3. Décision du prochain mouvement
        if not current_path:
            # Si pas de chemin, on cherche une nouvelle cible inexplorée
            target_pos = explorer.trouver_cible_lointaine()
            
            if target_pos is None:
                print("🏁 Exploration terminée (tout est vu).")
                tts.say("J'ai fini d'explorer.")
                break
                
            print(f"📍 Nouvelle cible algorithmique : {target_pos}")
            path = explorer.trouver_chemin_astar((seeker.x, seeker.y), target_pos)
            
            if path:
                current_path = path
            else:
                # Blocage : on fait une rotation sur place pour débloquer la vue/A*
                print("⚠️ Pas de chemin, rotation de secours.")
                motion_service.moveTo(0, 0, math.radians(45))
                seeker.tourner(45)
                continue

        # 4. Exécution du mouvement (Physique)
        if current_path:
            next_x, next_y = current_path.pop(0)
            
            # Calcul du déplacement en cases
            dx = next_x - seeker.x
            dy = next_y - seeker.y
            
            if dx == 0 and dy == 0:
                continue
                
            # Calcul de l'angle et de la distance pour le monde réel
            dist_meters = math.sqrt(dx**2 + dy**2) * CELL_SIZE
            
            # Orientation : Le robot doit se tourner vers la case cible
            # L'angle cible dans la grille (0° = Est, 90° = Sud dans votre code fonction_score ?)
            # Vérifions votre code : 0->(1,0) (Est), 90->(0,1) (Sud). C'est standard image.
            target_angle_rad = math.atan2(dy, dx)
            
            # Conversion de l'angle absolu grille en angle relatif robot
            # On assume que seeker.direction_angle est en degrés et absolu
            current_angle_rad = math.radians(seeker.direction_angle)
            rotation_needed = target_angle_rad - current_angle_rad
            
            # Normalisation de l'angle (-pi à pi)
            rotation_needed = (rotation_needed + math.pi) % (2 * math.pi) - math.pi
            
            # --- COMMANDE MOTEUR ---
            # On tourne d'abord (si nécessaire) puis on avance
            if abs(rotation_needed) > 0.1: # Si rotation significative
                motion_service.moveTo(0, 0, rotation_needed)
            
            motion_service.moveTo(dist_meters, 0, 0)
            
            # 5. Mise à jour de la position virtuelle
            seeker.x = next_x
            seeker.y = next_y
            # Mise à jour de l'angle virtuel (on arrondit comme dans votre algo)
            new_angle_deg = math.degrees(math.atan2(dy, dx))
            seeker.direction_angle = round(new_angle_deg / 45) * 45 % 360
            
            # (Optionnel) Vérification Sonar pour éviter les vrais murs non cartographiés
            l = memory_service.getData("Device/SubDeviceList/US/Left/Sensor/Value")
            r = memory_service.getData("Device/SubDeviceList/US/Right/Sensor/Value")
            if l < 0.4 or r < 0.4:
                print("🛑 Obstacle imprévu détecté !")
                motion_service.stopMove()
                # Ici, il faudrait idéalement mettre à jour la grille (grid_map) avec un mur
                # explorer.knowledge_map[ny][nx] = 2 
                current_path = [] # On force le recalcul d'un chemin

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