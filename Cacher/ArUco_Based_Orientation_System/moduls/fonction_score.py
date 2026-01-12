import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
import json
import time
from collections import deque 
import heapq

# =======================================================================================
#   l'afichage de la carte et le deplacement du seeker
# =======================================================================================

class Seeker:
    def __init__(self, pos, vision_angle, max_distance, direction_angle, grid, target, size):
        self.x, self.y = pos
        self.vision_angle = vision_angle
        self.max_distance = max_distance
        self.direction_angle = direction_angle
        self.seeker_size = size
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.target = target
        self.cos_half_vision = math.cos(math.radians(self.vision_angle / 2))
        self.direction_map_xy = {
            0: (1, 0), 90: (0, 1), 180: (-1, 0), 270: (0, -1),
            45: (1, 1), 135: (-1, 1), 225: (-1, -1), 315: (1, -1)
        }

    def in_vision(self, px, py):
        dx = px - self.x
        dy = py - self.y
        dist_sq = dx*dx + dy*dy
        if dist_sq > self.max_distance**2:
            return False
        
        # Calcul de la direction du vecteur vision
        rad = math.radians(self.direction_angle)
        dir_x = math.cos(rad)
        dir_y = math.sin(rad)
        
        # Calcul du produit scalaire (pour vérifier l'angle)
        dot = dx*dir_x + dy*dir_y
        
        if dot < 0: 
            return False

        if dot < math.sqrt(dist_sq) * self.cos_half_vision:
            return False
        
        # Vérification du Ray-Casting (ligne de vue)
        steps = max(abs(dx), abs(dy))
        if steps == 0:
            return True

        for i in range(1, steps): 
            x_check = int(round(self.x + dx * i / steps))
            y_check = int(round(self.y + dy * i / steps))
            
            # Vérification des murs: grid[ligne][colonne] -> grid[y][x]
            if 0 <= y_check < self.rows and 0 <= x_check < self.cols and self.grid[y_check][x_check] == 1:
                return False
        
        # Vérifier la dernière cellule elle-même (le mur ou la cible)
        if 0 <= py < self.rows and 0 <= px < self.cols and self.grid[py][px] == 1:
            return False
            
        return True

    def avancer(self):
        dx, dy = self.direction_map_xy.get(self.direction_angle, (0,0))
        nx, ny = self.x + dx, self.y + dy
        
        # Vérification physique des obstacles (murs ou bordures)
        for i in range(-self.seeker_size//2, self.seeker_size//2):
            for j in range(-self.seeker_size//2, self.seeker_size//2):
                xi, yj = nx + i, ny + j
                
                # Vérification des limites et murs (y, x)
                if not (0 <= yj < self.rows and 0 <= xi < self.cols) or self.grid[yj][xi] == 1:
                    return False
                    
        self.x, self.y = nx, ny
        return True
    
    def tourner(self, angle=45): 
        self.direction_angle = (self.direction_angle + angle) % 360

    def cherche(self):
        # La cible est un ensemble de coordonnées (x, y)
        for (i,j) in self.target:
            if self.in_vision(i,j):
                return True
        return False

    def cases_en_vision(self):
        visible = []
        for y_idx in range(self.rows):
            for x_idx in range(self.cols):
                if self.in_vision(x_idx, y_idx): # (x, y) -> (colonne, ligne)
                    visible.append((x_idx, y_idx))
        return visible

    def avancer_possible(self, dx, dy):
        nx, ny = self.x + dx, self.y + dy
        
        for i in range(-self.seeker_size//2, self.seeker_size//2):
            for j in range(-self.seeker_size//2, self.seeker_size//2):
                xi, yj = nx + i, ny + j
                if not (0 <= yj < self.rows and 0 <= xi < self.cols) or self.grid[yj][xi] == 1:
                    return False
        return True

# =======================================================================================
#   Classe pour la Logique de Recherche et de Pathfinding
# =======================================================================================

class Explorer:
    def __init__(self, seeker):
        self.seeker = seeker
        # 0: non vu, 1: vu (dans cône), 2: mur
        self.knowledge_map = np.zeros((seeker.rows, seeker.cols), dtype=int)
        for y in range(seeker.rows):
            for x in range(seeker.cols):
                if seeker.grid[y][x] == 1:
                    self.knowledge_map[y][x] = 2 # Marquer les murs comme 2 (Obstacle)

    def mettre_a_jour_vue(self):
        visible_cells = self.seeker.cases_en_vision()
        for x, y in visible_cells:
            # knowledge_map[ligne][colonne] -> [y][x]
            if self.knowledge_map[y][x] != 2:
                self.knowledge_map[y][x] = 1

    def trouver_cible_lointaine(self):
        max_dist_sq = -1
        best_target = None
        
        for y in range(self.seeker.rows):
            for x in range(self.seeker.cols):
                if self.knowledge_map[y][x] == 0:
                    dist_sq = (x - self.seeker.x)**2 + (y - self.seeker.y)**2
                    if dist_sq > max_dist_sq:
                        
                        # Vérification de l'accessibilité de la zone pour le corps du robot
                        is_valid_area = True
                        for i in range(-self.seeker.seeker_size//2, self.seeker.seeker_size//2):
                            for j in range(-self.seeker.seeker_size//2, self.seeker.seeker_size//2):
                                xi, yj = x + i, y + j
                                # Grille indexée par (ligne, colonne) -> (y, x)
                                if not (0 <= yj < self.seeker.rows and 0 <= xi < self.seeker.cols) or self.seeker.grid[yj][xi] == 1:
                                    is_valid_area = False
                                    break
                            if not is_valid_area:
                                break

                        if is_valid_area:
                            max_dist_sq = dist_sq
                            best_target = (x, y)
                            
        return best_target # (x, y) du point le plus lointain et non vu

    def trouver_chemin_astar(self, start_pos, target_pos):
        # start_pos et target_pos sont (x, y)
        if target_pos is None:
            return None
        
        start_x, start_y = int(start_pos[0]), int(start_pos[1])
        target_x, target_y = int(target_pos[0]), int(target_pos[1])
        
        # File de priorité: (f_score, x, y)
        # f_score = g_score (coût réel) + h_score (heuristique)
        open_list = [(0, start_x, start_y)] 
        
        # g_score: coût réel pour aller du départ au nœud (x, y)
        g_score = {(start_x, start_y): 0}
        
        # parent: Pour reconstruire le chemin
        parent = {(start_x, start_y): None}
        
        # Directions: 8 mouvements (dx, dy)
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        rows, cols = self.seeker.rows, self.seeker.cols
        seeker_size = self.seeker.seeker_size
        
        # Coûts de traversée basés sur knowledge_map[y][x]:
        COST_UNSEEN = 1       # Coût normal pour les cases non vues (0)
        COST_SEEN_PENALTY = 5 # Coût pénalisant pour les cases vues (1)
        
        while open_list:
            # Récupérer le nœud avec le plus petit f_score
            f, curr_x, curr_y = heapq.heappop(open_list)
            current_pos = (curr_x, curr_y)
            
            if curr_x == target_x and curr_y == target_y:
                # Reconstruire le chemin
                path = []
                while parent[current_pos] is not None:
                    path.append(current_pos)
                    current_pos = parent[current_pos]
                path.reverse()
                return path

            for dx, dy in directions:
                next_x, next_y = curr_x + dx, curr_y + dy
                next_pos = (next_x, next_y)
                
                # 1. Vérifier si le mouvement est valide (pas de mur/bordure)
                is_valid = True
                for i in range(-seeker_size//2, seeker_size//2):
                    for j in range(-seeker_size//2, seeker_size//2):
                        xi, yj = next_x + i, next_y + j
                        # Grille indexée par (ligne, colonne) -> (y, x)
                        if not (0 <= yj < rows and 0 <= xi < cols) or self.seeker.grid[yj][xi] == 1:
                            is_valid = False
                            break
                    if not is_valid:
                        break
                
                if not is_valid:
                    continue
                
                # 2. Déterminer le coût de traversée (pénalité)
                # Utiliser la knowledge_map[y][x]
                if self.knowledge_map[next_y][next_x] == 0:
                    cost = COST_UNSEEN
                elif self.knowledge_map[next_y][next_x] == 1:
                    cost = COST_SEEN_PENALTY
                elif self.knowledge_map[next_y][next_x] == 2:
                    continue # Mur, déjà filtré par is_valid mais par sécurité
                else:
                    cost = COST_SEEN_PENALTY # Par défaut si autre valeur inattendue
                
                # Coût du mouvement (1 pour orthogonaux, sqrt(2) pour diagonaux, plus le coût de pénalité)
                # On utilise simplement 1 + coût de pénalité pour simplifier les calculs de g_score (pour BFS/grille)
                # Pour les diagonales, on peut pénaliser davantage ou utiliser sqrt(2)
                move_cost = math.sqrt(dx*dx + dy*dy)
                
                # Le coût total pour arriver à next_pos
                new_g_score = g_score[current_pos] + move_cost * cost
                
                # 3. Vérifier si ce nouveau chemin est meilleur
                if next_pos not in g_score or new_g_score < g_score[next_pos]:
                    g_score[next_pos] = new_g_score
                    parent[next_pos] = current_pos
                    
                    # Heuristique (distance euclidienne)
                    h_score = math.sqrt((next_x - target_x)**2 + (next_y - target_y)**2)
                    f_score = new_g_score + h_score
                    
                    heapq.heappush(open_list, (f_score, next_x, next_y))
                    
        return None # Pas de chemin trouvé

# =======================================================================================
#   Algorithme de Recherche et Affichage (MODIFIÉ pour la terminaison propre)
# =======================================================================================

def recherche_affichage(seeker, step_limit, mise_a_jour_affichage):
    steps = 0
    explorer = Explorer(seeker)
    current_path = [] 

    while True:
        if steps > step_limit:
            return None

        # 1. Mise à jour de la carte de connaissance
        explorer.mettre_a_jour_vue()
        
        # 2. Vérifier si la cible est trouvée (priorité absolue)
        if seeker.cherche():
            mise_a_jour_affichage(seeker, explorer.knowledge_map, current_path)
            # NOUVEAU : Arrêt immédiat et fermeture de la fenêtre
            plt.close('all') 
            return steps

        # 3. Mouvement du robot (Basé sur le chemin ou la recherche d'un nouveau)
        if not current_path:
            # 3.1. Trouver la nouvelle cible la plus lointaine
            target_pos = explorer.trouver_cible_lointaine()
            
            if target_pos is None:
                print("Exploration terminée : Aucune case non vue restante.")
                mise_a_jour_affichage(seeker, explorer.knowledge_map, current_path)
                plt.close('all')
                return None
            
            # 3.2. Calculer le chemin le plus court vers cette cible
            start_pos = (seeker.x, seeker.y)
            path = explorer.trouver_chemin_astar(start_pos, target_pos)
            
            if path:
                current_path = path
                print(f"Nouveau chemin trouvé vers le point ({target_pos[0]}, {target_pos[1]}) de longueur {len(path)}.")
            else:
                # Si pas de chemin trouvé, on tourne et on réessaie à la prochaine itération
                print(f"Impossible de trouver un chemin vers ({target_pos[0]}, {target_pos[1]}). Tentative de rotation.")
                seeker.tourner(45) 
                steps += 1
                mise_a_jour_affichage(seeker, explorer.knowledge_map, current_path)
                continue 

        # 4. Avancer le long du chemin
        if current_path:
            next_x, next_y = current_path.pop(0) 
            
            # Mise à jour de la direction du seeker
            dx, dy = next_x - seeker.x, next_y - seeker.y
            if dx != 0 or dy != 0:
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)
                seeker.direction_angle = round(angle_deg / 45) * 45 % 360

            # Déplacer le seeker
            seeker.x, seeker.y = next_x, next_y
            steps += 1

        # 5. Affichage
        mise_a_jour_affichage(seeker, explorer.knowledge_map, current_path)
        
        # Vérification si le robot est bloqué
        if not current_path and explorer.trouver_cible_lointaine() is None:
             plt.close('all')
             return steps


# =======================================================================================
#   Fonction Score et Initialisation (MODIFIÉ pour la terminaison propre)
# =======================================================================================

def score(x, y, grid, show_display=False, step_limit=5000, taille_seeker=30, angle_vision=70, distance_vision=300):

    if show_display:
        fig, ax = plt.subplots()
        cmap = colors.ListedColormap([
            'white',  # 0 (Non vu - Inconnu)
            'lightgray',  # 1 (Vu - Exploré)
            'black',  # 2 (Mur)
            'orange', # 3 (Seeker)
            'blue', # 4 (Target)
            'yellow', # 5 (Vision actuelle)
            'red'  # 6 (Chemin planifié)
        ])
        bounds = [-0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5]
        norm = colors.BoundaryNorm(bounds, cmap.N)
        im = ax.imshow(grid, cmap=cmap, norm=norm, origin='lower')
        plt.ion()
        plt.show()

        def mise_a_jour_affichage(seeker, knowledge_map, current_path):
            vis_grid = np.copy(knowledge_map) 
            
            for (i,j) in seeker.cases_en_vision():
                if vis_grid[j][i] != 2 and vis_grid[j][i] != 4: 
                    vis_grid[j][i] = 5 
                    
            for path_x, path_y in current_path:
                if vis_grid[path_y][path_x] != 5: 
                    vis_grid[path_y][path_x] = 6 

            for i, j in seeker.target:
                 vis_grid[j][i] = 4
                 
            xs = int(seeker.x)
            ys = int(seeker.y)
            for i in range(-seeker.seeker_size//2, seeker.seeker_size//2):
                for j in range(-seeker.seeker_size//2, seeker.seeker_size//2):
                    xi, yj = xs + i, ys + j
                    if 0 <= yj < vis_grid.shape[0] and 0 <= xi < vis_grid.shape[1]:
                        vis_grid[yj, xi] = 3 

            im.set_data(vis_grid)
            plt.draw()
            plt.pause(0.01)

    # --- Initialisation de la cible (x=col, y=row) ---
    targets = []
    # Créer une copie de la grille des murs pour l'affichage de la cible (4)
    temp_grid = np.copy(grid) 
    rows, cols = temp_grid.shape
    
    # Vérification des limites
    x_min = x - taille_seeker // 2
    x_max = x + taille_seeker // 2
    y_min = y - taille_seeker // 2
    y_max = y + taille_seeker // 2

    if x_min < 0 or x_max >= cols or y_min < 0 or y_max >= rows:
        print(f"Erreur : La cible de centre ({x}, {y}) et de taille {taille_seeker} dépasse les limites ({rows}, {cols}).")
        return None 
        
    for i in range(-taille_seeker//2, taille_seeker//2):
        for j in range(-taille_seeker//2, taille_seeker//2):
            targets.append((x+i, y+j)) 
            temp_grid[y+j][x+i] = 4 # Marquer la cible

    seeker = Seeker(
        pos=(67, 5 + (taille_seeker//2)), # Position de départ du chercheur
        vision_angle=angle_vision,
        max_distance=distance_vision,
        direction_angle=0,
        grid=grid, # La grille originale (murs seulement)
        target=targets,
        size=taille_seeker
    )

    if show_display:
        steps = recherche_affichage(seeker, step_limit, mise_a_jour_affichage)
        plt.ioff()
        # Le plt.show() final est retiré pour ne pas bloquer
        return steps
    else:
        # Vous devez adapter l'Explorer pour un mode sans affichage si nécessaire
        return None 


# =======================================================================================
#   Test et affichage (MODIFIÉ pour les coordonnées valides)
# =======================================================================================

with open("./Recherche/maps/carte_03.json", "r") as f:
    grid = np.array(json.load(f))


carte = np.array(grid) 

facteur = 4
nouvelle_taille_row = carte.shape[0] // facteur
nouvelle_taille_col = carte.shape[1] // facteur

carte_reduite = np.zeros((nouvelle_taille_row, nouvelle_taille_col), dtype=int)

for i in range(nouvelle_taille_row):
    for j in range(nouvelle_taille_col):
        bloc = carte[i*facteur:(i+1)*facteur, j*facteur:(j+1)*facteur]
        carte_reduite[i, j] = 1 if np.any(bloc == 1) else 0

# Coordonnées de test qui fonctionnent sur la carte réduite (87x177)
TEST_TARGET_X = 60  # Colonne (X) valide
TEST_TARGET_Y = 120 # Ligne (Y) valide

print(f"Lancement de la recherche sur la carte réduite ({nouvelle_taille_row}x{nouvelle_taille_col})...")

result_steps = score(
    x=TEST_TARGET_X,
    y=TEST_TARGET_Y,
    grid=carte_reduite, 
    show_display=True, 
    taille_seeker=30//facteur, 
    distance_vision=300//facteur
)

if result_steps is not None:
    print(f"\n✅ Le robot a trouvé la cible en {result_steps} pas.")
else:
    print("\n❌ Le robot n'a pas trouvé la cible dans la limite de pas ou a terminé l'exploration.")