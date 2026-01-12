import random
import numpy as np
import os
import json

# --- Formes correctement dessinées ---
def forme_rectangle(w, h):
    return np.ones((h, w), dtype=int)

def forme_L(w, h):
    """L rempli, orientation 0 par défaut"""
    mat = np.zeros((h, w), dtype=int)
    ep = max(1, min(w, h)//3)
    mat[:, :ep] = 1  # barre verticale
    mat[-ep:, :] = 1  # barre horizontale
    return mat

def forme_U(w, h):
    """U reconnaissable avec 1 sur le contour et vide à l'intérieur"""
    mat = np.zeros((h, w), dtype=int)
    ep = max(1, w//5)
    mat[:, :ep] = 1  # côté gauche
    mat[:, -ep:] = 1 # côté droit
    mat[-ep:, :] = 1 # barre du bas
    return mat

def forme_T(w, h):
    """T reconnaissable avec barre horizontale en haut et verticale au milieu"""
    mat = np.zeros((h, w), dtype=int)
    ep_h = max(1, h//5)
    ep_w = max(1, w//5)
    mat[:ep_h, :] = 1           # barre horizontale en haut
    mat[:, w//2 - ep_w//2 : w//2 + ep_w//2 + 1] = 1  # barre verticale au milieu
    return mat

formes_base = {
    "rectangle": forme_rectangle,
    "L": forme_L,
    "U": forme_U,
    "T": forme_T
}

def rotation_aleatoire(obj):
    """Rotation aléatoire pour L, T, U"""
    k = random.randint(0, 3)
    return np.rot90(obj, k)

def generer_carte(
    width=350,
    height=710,
    nb_obj_min=8, nb_obj_max=8,
    obj_w_min=50, obj_w_max=150,
    obj_h_min=50, obj_h_max=150,
    min_gap=31,
    max_attempts=300
):
    carte = np.zeros((height, width), dtype=int)

    # Bordures
    carte[0, :] = 1
    carte[-1, :] = 1
    carte[:, 0] = 1
    carte[:, -1] = 1

    # Porte
    porte_x = 220
    porte_w = 95
    carte[0, porte_x:porte_x + porte_w] = 2

    nb_objets = random.randint(nb_obj_min, nb_obj_max)

    for obj_index in range(nb_objets):
        placed = False
        attempts = 0

        while not placed and attempts < max_attempts:
            attempts += 1

            forme_name = random.choice(list(formes_base.keys()))
            ow = random.randint(obj_w_min, min(obj_w_max, width - 2))
            oh = random.randint(obj_h_min, min(obj_h_max, height - 2))
            obj = formes_base[forme_name](ow, oh)

            if forme_name in ["L", "T", "U"]:
                obj = rotation_aleatoire(obj)

            # Limites pour éviter de coller les murs
            x_min = 1 + min_gap
            y_min = 1 + min_gap
            x_max = width - obj.shape[1] - 1 - min_gap
            y_max = height - obj.shape[0] - 1 - min_gap

            if x_max <= x_min or y_max <= y_min:
                continue

            x = random.randint(x_min, x_max)
            y = random.randint(y_min, y_max)

            # Vérification avec marge pour les autres obstacles
            check_x0 = max(1, x - min_gap)
            check_x1 = min(width - 1, x + obj.shape[1] + min_gap)
            check_y0 = max(1, y - min_gap)
            check_y1 = min(height - 1, y + obj.shape[0] + min_gap)

            if np.any(carte[check_y0:check_y1, check_x0:check_x1] != 0):
                continue

            # Placement
            carte[y:y+obj.shape[0], x:x+obj.shape[1]] += obj

            inner = carte[1:-1, 1:-1]
            if np.any(np.all(inner >= 1, axis=1)) or np.any(np.all(inner >= 1, axis=0)):
                carte[y:y+obj.shape[0], x:x+obj.shape[1]] -= obj
                continue

            placed = True

        if not placed:
            print(f"⚠️ Impossible de placer l'objet #{obj_index+1} après {max_attempts} essais.")

    # Zone libre au début
    for j in range(1, 75):
        carte[j, 1:width-1] = 0

    return carte

def sauvegarder_cartes(n=5, dossier="./maps", seed=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    if not os.path.exists(dossier):
        os.makedirs(dossier)

    for i in range(1, n + 1):
        carte = generer_carte()
        fichier = os.path.join(dossier, f"carte_{i:02d}.json")
        with open(fichier, "w") as f:
            json.dump(carte.tolist(), f, indent=2)
        print(f"✅ Carte {i} sauvegardée → {fichier}")

if __name__ == "__main__":
    sauvegarder_cartes(10, dossier="./maps", seed=42)