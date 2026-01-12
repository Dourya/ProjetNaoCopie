import argparse
import json
import os
import sys
import matplotlib.pyplot as plt
import numpy as np

def charger_carte(fichier):
    """
    Charge une carte JSON depuis un fichier.
    """
    with open(fichier, "r") as f:
        return np.array(json.load(f))


def afficher_carte_fenetre(carte, nom_carte):
    """
    Affiche une carte 2D dans une fenêtre graphique.
    1 = obstacle/mur → noir
    0 = vide → blanc
    """
    plt.imshow(carte, cmap="gray_r", origin="upper")
    plt.title(f"Carte : {nom_carte}")
    plt.axis("equal")
    plt.xticks([])
    plt.yticks([])
    plt.show()


if __name__ == "__main__":
# Vérification manuelle pour le message d'erreur personnalisé demandé
    if len(sys.argv) < 2:
        print("Erreur : Un paramètre (le chemin vers la carte) est obligatoire.")
        sys.exit(1)

    # Configuration de l'argument via argparse
    parser = argparse.ArgumentParser(description="Affiche une carte JSON avec Matplotlib.")
    parser.add_argument("chemin_carte", help="Le chemin vers le fichier .json de la carte")
    args = parser.parse_args()

    fichier_selectionne = args.chemin_carte

    # Vérification que le fichier existe
    if not os.path.exists(fichier_selectionne):
        print(f"Erreur : Le fichier '{fichier_selectionne}' n'existe pas.")
        sys.exit(1)

    # Charger et afficher la carte
    carte = charger_carte(fichier_selectionne)
    
    # On utilise le nom du fichier (sans le chemin complet) pour le titre de la fenêtre
    nom_carte = os.path.basename(fichier_selectionne)
    
    afficher_carte_fenetre(carte, nom_carte)
