import json
import os
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

def charger_carte(fichier):
    """
    Charge une carte JSON depuis un fichier.
    """
    with open(fichier, "r") as f:
        return np.array(json.load(f))


def afficher_carte_fenetre(carte, nom_carte):
    """
    Affiche une carte 2D dans une fenêtre graphique.
    1 = obstacle/mur -> noir
    0 = vide -> blanc
    2 = porte -> vert
    """
    cmap = ListedColormap(["white", "black", "green"])
    plt.imshow(carte, cmap=cmap, origin="lower")
    plt.title(f"Carte : {nom_carte}")
    plt.axis("equal")
    plt.xticks([])
    plt.yticks([])
    plt.show()


if __name__ == "__main__":
    dossier = "./Recherche/maps"
    fichiers = sorted([f for f in os.listdir(dossier) if f.endswith(".json")])

    if not fichiers:
        print("Aucune carte trouvée dans le dossier './maps'.")
        exit()

    # Choisir une carte à afficher
    print("Cartes disponibles :")
    for i, f in enumerate(fichiers, 1):
        print(f"{i}. {f}")

    choix = int(input("Numéro de la carte à afficher : ")) - 1
    fichier_selectionne = os.path.join(dossier, fichiers[choix])

    # Charger et afficher la carte
    carte = charger_carte(fichier_selectionne)
    afficher_carte_fenetre(carte, fichiers[choix])