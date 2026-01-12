# vision/detection.py
import cv2
import numpy as np
from tensorflow.keras.models import load_model

def load_classifier(model_path, labels_path):
    """
    Charge le modèle Keras et les labels.
    """
    model = load_model(model_path, compile=False)
    with open(labels_path, "r") as f:
        class_names = [line.strip() for line in f.readlines()]
    return model, class_names


def preprocess_image(img):
    """
    Prépare l'image pour la prédiction (224x224, normalisation [-1,1]).
    """
    if img is None or img.size == 0:
        return None

    img_resized = cv2.resize(img, (224, 224), interpolation=cv2.INTER_AREA)
    img_resized = np.asarray(img_resized, dtype=np.float32).reshape(1, 224, 224, 3)
    return (img_resized / 127.5) - 1.0


def detect_nao(img, model, class_names, threshold=0.95, target_keywords=("nao", "robot")):
    """
    Lance la détection sur une image.
    Retourne (True, class_name, confidence) si un NAO est détecté.
    """
    if img is None:
        return False, None, 0.0

    img_prepared = preprocess_image(img)
    if img_prepared is None:
        return False, None, 0.0

    prediction = model.predict(img_prepared, verbose=0)
    index = int(np.argmax(prediction))
    class_name = class_names[index] if index < len(class_names) else "Inconnu"
    confidence = float(prediction[0][index])

    # 🔎 Recherche d’un mot-clé cible ("nao", "robot", etc.)
    detected = any(key.lower() in class_name.lower() for key in target_keywords) and confidence >= threshold

    return detected, class_name, confidence
