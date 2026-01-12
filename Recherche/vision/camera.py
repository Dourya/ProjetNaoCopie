# vision/camera.py

import cv2
import numpy as np
import time # <-- AJOUTÉ pour la robustesse

def subscribe_camera(video_service, resolution=2, color_space=11, fps=15, camera_index=1):
    """
    S'abonne à une caméra NAO spécifique.
    Retourne l'ID d'abonnement si réussi, sinon None.
    """
    try:
        # Désabonne les anciens abonnés liés à la même caméra
        for name in video_service.getSubscribers():
            try:
                video_service.unsubscribe(name)
            except Exception:
                pass
        
        # 💡 AJOUT : Laisse le service vidéo se stabiliser après la désinscription
        time.sleep(0.1) 

        name_id = video_service.subscribeCamera(
            f"nao_camera_{camera_index}", camera_index, resolution, color_space, fps
        )
        return name_id
    except Exception as e:
        print(f"[⚠️] Erreur lors de l’abonnement à la caméra {camera_index}: {e}")
        return None


def get_frame(video_service, name_id):
    """
    Capture une image de la caméra NAO et retourne un tableau numpy (BGR).
    Gère la conversion YUV422 (9) et RGB (11) vers BGR.
    Retourne None si la capture échoue.
    """
    if not name_id:
        return None

    try:
        image = video_service.getImageRemote(name_id)
        if image is None:
            return None

        width, height, layers, color_space = image[0], image[1], image[2], image[3]
        array = image[6]
        
        # Conversion d'espace couleur
        if color_space == 9: # YUV422
            # YUV422 -> BGR
            arr = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 2))
            img = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)
        elif color_space == 11: # RGB
            # RGB -> BGR
            img = np.frombuffer(array, dtype=np.uint8).reshape((height, width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        else:
            print(f"[⚠️] Espace couleur inconnu : {color_space}")
            return None 

        return img
    except Exception as e:
        print(f"[⚠️] Erreur lors de la capture image ({name_id}): {e}")
        return None


def unsubscribe_camera(video_service, name_id):
    """
    Libère proprement l'abonnement à la caméra.
    """
    if not name_id:
        return
    try:
        video_service.unsubscribe(name_id)
    except Exception as e:
        print(f"[⚠️] Impossible de désabonner {name_id}: {e}")