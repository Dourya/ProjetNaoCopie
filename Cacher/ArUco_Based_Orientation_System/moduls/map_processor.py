import threading
import queue
import json
import numpy as np
import math
import heapq
import random
import time

# Imports Matplotlib optimisés pour le threading (Backend Agg = pas de fenêtre popup)
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.patches import Circle

class MapProcessor:
    def __init__(self):
        self.command_queue = queue.Queue()
        self.result_queue = queue.Queue()
        self.running = False
        self.thread = None
        # État interne
        self.grid = None
        self.rows = 0
        self.cols = 0
        # --- MODIFICATION ---
        # Stockage des robots : { id: {'x': val, 'y': val, 'valid': bool} }
        self.robots = {} 
        # --------------------
        self.target_pos = None  
        self.current_path = []

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_loop)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def send_command(self, cmd, data=None):
        """Méthode helper pour envoyer des ordres depuis l'interface"""
        self.command_queue.put((cmd, data))

    def _run_loop(self):
        while self.running:
            try:
                cmd, data = self.command_queue.get(timeout=0.1)

                if cmd == "LOAD_MAP":
                    self._load_map(data)
                
                elif cmd == "FIND_HIDING_SPOT":
                    self._find_random_target()
                
                elif cmd == "CALCULATE_PATH":
                    self._calculate_astar()
                
                # --- NOUVELLE COMMANDE ---
                elif cmd == "UPDATE_ROBOT_POS":
                    self._update_robot_positions(data)
                # -------------------------

                self._render_map()

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Erreur MapProcessor : {e}")

    def _update_robot_positions(self, markers_data):
        """Met à jour la position des robots avec décalage (Centrage)."""
        if self.grid is None: return

        # Calcul du centre de la carte (l'offset à appliquer)
        center_x = self.cols // 2
        center_y = self.rows // 2

        for m in markers_data:
            mid = m['id']
            
            # 1. Récupération + TRANSFORMATION
            # On ajoute la moitié de la carte pour que (0,0) caméra devienne le centre carte
            raw_x = int(m['x']) + center_x
            raw_y = int(m['y']) + center_y

            # 2. Vérification des limites (sur les nouvelles coordonnées)
            is_inside = (0 <= raw_x < self.cols) and (0 <= raw_y < self.rows)

            if mid not in self.robots:
                # Nouveau robot
                self.robots[mid] = {'x': raw_x, 'y': raw_y, 'valid': is_inside}
            else:
                # Robot existant
                if is_inside:
                    self.robots[mid]['x'] = raw_x
                    self.robots[mid]['y'] = raw_y
                    self.robots[mid]['valid'] = True
                else:
                    # Hors limites : On marque invalide, mais on garde la dernière position connue 
                    # (ou vous pouvez mettre à jour raw_x/raw_y si vous voulez voir le point gris bouger hors map)
                    self.robots[mid]['valid'] = False
                    # Optionnel : self.robots[mid]['x'] = raw_x (si vous voulez voir où il est "hors cadre")

    # --- LOGIQUE MÉTIER ---

    def _load_map(self, filepath):
        try:
            with open(filepath, 'r') as f:
                self.grid = np.array(json.load(f)).astype(int)
                self.rows, self.cols = self.grid.shape
                # Reset
                self.target_pos = None
                self.current_path = []
                #print("Carte chargée avec succès dans le thread.")
        except Exception as e:
            print(f"Erreur chargement JSON: {e}")

    def _find_random_target(self):
        """Cherche une cachette aléatoire (case vide = 0)"""
        if self.grid is None: return

        # Trouver toutes les cases vides (0)
        # argwhere renvoie [row, col], donc [y, x]
        empty_spots = np.argwhere(self.grid == 0)
        
        if len(empty_spots) > 0:
            choice = random.choice(empty_spots)
            # On stocke en (x, y) pour être cohérent avec l'affichage
            self.target_pos = (choice[1], choice[0]) 
            #print(f"Nouvelle cible aléatoire : {self.target_pos}")
        else:
            print("Aucune case libre trouvée !")

    def _is_safe(self, x, y, radius=18):
        """
        Vérifie si le robot peut se tenir en (x, y) sans toucher un mur.
        On vérifie une zone carrée autour du point central.
        radius = 18 correspond à un diamètre de 36cm (Sécurité pour les 35cm du robot)
        """
        # Définition de la boite de collision (Bounding Box)
        x_min = max(0, x - radius)
        x_max = min(self.cols, x + radius)
        y_min = max(0, y - radius)
        y_max = min(self.rows, y + radius)

        # Extraction de la sous-grille (Slicing Numpy)
        sub_grid = self.grid[y_min:y_max, x_min:x_max]

        # Si un seul pixel de cette zone vaut 1 (mur), ce n'est pas sûr
        if np.any(sub_grid == 1):
            return False
        return True

    def _calculate_astar(self, robot_id=None):
        """
        Logique A* prenant en compte la taille du robot.
        robot_id : (int/str) L'ID du robot à déplacer. Si None, prend le premier valide.
        """
        if self.grid is None or self.target_pos is None:
            return

        # --- 1. DÉTERMINATION DU POINT DE DÉPART ---
        actual_start_pos = None

        # Cas A : Un ID spécifique est demandé
        if robot_id is not None:
            if robot_id in self.robots and self.robots[robot_id]['valid']:
                r = self.robots[robot_id]
                actual_start_pos = (r['x'], r['y'])
                print(f"🗺️ Pathfinding depuis le Robot ID {robot_id}")
            else:
                print(f"❌ Erreur : Robot ID {robot_id} introuvable ou hors limites.")
                return

        # Cas B : Aucun ID, on prend le premier robot valide disponible
        else:
            # On cherche s'il y a des robots valides détectés
            for rid, r in self.robots.items():
                if r['valid']:
                    actual_start_pos = (r['x'], r['y'])
                    print(f"🗺️ Pathfinding depuis le Robot ID {rid} (Par défaut)")
                    break
            
            # Cas C : Aucun robot détecté, on utilise la position de debug (self.start_pos)
            if actual_start_pos is None:
                print("⚠️ Aucun robot détecté, utilisation de la position par défaut (Debug).")
                actual_start_pos = self.start_pos

        # Conversion en entiers pour la grille
        start_node = (int(actual_start_pos[0]), int(actual_start_pos[1]))
        end_node = (int(self.target_pos[0]), int(self.target_pos[1]))

        # --- 2. VÉRIFICATIONS DE SÉCURITÉ ---
        
        # Vérification du point de départ
        if not self._is_safe(start_node[0], start_node[1]):
            print(f"❌ Erreur : Le point de départ {start_node} est dans (ou trop près) d'un mur !")
            return
        
        # Vérification du point d'arrivée
        if not self._is_safe(end_node[0], end_node[1]):
            print(f"❌ Erreur : La destination {end_node} est trop proche d'un mur !")
            return

        # --- 3. ALGORITHME A* ---
        open_list = [(0, start_node)]
        came_from = {}
        g_score = {start_node: 0}
        
        found = False

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == end_node:
                found = True
                break

            # 8 Directions (Diagonales incluses)
            for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Vérif limites carte
                if 0 <= neighbor[1] < self.rows and 0 <= neighbor[0] < self.cols:
                    
                    # Vérification de la zone de sécurité (taille du robot)
                    if self._is_safe(neighbor[0], neighbor[1]):
                        
                        # Coût du déplacement (1 pour adjacent, 1.414 pour diagonale)
                        move_cost = math.hypot(dx, dy)
                        tentative_g = g_score[current] + move_cost
                        
                        if neighbor not in g_score or tentative_g < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g
                            # Heuristique (Distance Euclidienne)
                            h_score = math.hypot(end_node[0]-neighbor[0], end_node[1]-neighbor[1])
                            heapq.heappush(open_list, (tentative_g + h_score, neighbor))

        if found:
            path = []
            curr = end_node
            while curr in came_from:
                path.append(curr)
                curr = came_from[curr]
            path.append(start_node)
            self.current_path = path[::-1] # Inverser
            print(f"✅ Chemin sécurisé trouvé : {len(path)} étapes.")
        else:
            print("❌ Pas de chemin trouvé (Obstacle infranchissable ?).")

    # --- RENDU GRAPHIQUE (MATPLOTLIB) ---
    def _render_map(self):
        if self.grid is None: return

        fig = Figure(figsize=(5, 4), dpi=100)
        ax = fig.add_subplot(111)
        
        # 1. Grille
        ax.imshow(self.grid, cmap='Greys', origin='upper')
        
        # 2. Robots (Boucle sur le dictionnaire)
        for mid, data in self.robots.items():
            rx, ry = data['x'], data['y']
            
            if data['valid']:
                # A. La Zone de Sécurité (Cercle physique de rayon 18)
                circle = Circle((rx, ry), radius=18, color='blue', alpha=0.3)
                ax.add_patch(circle)
                
                # B. Le Centre précis du robot (petit point)
                ax.plot(rx, ry, marker='o', color='blue', markersize=4)
                ax.text(rx, ry, f"{mid}", color='black', fontsize=7, ha='center', va='center', fontweight='bold')
                
            else:
                # Robot hors limites (Gris)
                circle = Circle((rx, ry), radius=18, color='gray', alpha=0.2)
                ax.add_patch(circle)
                
                ax.plot(rx, ry, marker='o', color='gray', markersize=4)
                ax.text(rx, ry-22, f"{mid} (Lost)", color='black', fontsize=8, ha='center')
        
        # 3. Cible
        if self.target_pos:
            ax.plot(self.target_pos[0], self.target_pos[1], 'rx', markersize=12, markeredgewidth=2)

        # 4. Chemin
        if self.current_path:
            xs = [p[0] for p in self.current_path]
            ys = [p[1] for p in self.current_path]
            ax.plot(xs, ys, 'g-', linewidth=2, alpha=0.7)

        ax.axis('off')
        fig.tight_layout(pad=0)

        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        X = np.asarray(buf)
        rgb_image = X[:, :, :3]
        
        self.result_queue.put(rgb_image)
        fig.clf()