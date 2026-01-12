# vision/__init__.py
from .camera import get_frame
from .detection import detect_nao

__all__ = ["get_frame", "detect_nao"]
