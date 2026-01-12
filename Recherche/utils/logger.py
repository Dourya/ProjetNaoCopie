# utils/logger.py
import logging
import sys

def setup_logger(name="NAO", level=logging.INFO):
    """
    Configure un logger simple pour le projet.
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Formatter
    formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')

    # Console handler
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(level)
    ch.setFormatter(formatter)

    # Ajouter handler si pas déjà présent
    if not logger.handlers:
        logger.addHandler(ch)

    return logger
