# vision/cam_tools.py
# Outils caméra : abonnement avec résolution ciblée, aperçu rapide couleur, et diagnostic.

import time
import cv2
import numpy as np

# --- Abonnement intelligent avec contrôle de résolution ---
def subscribe_with_target_res(video_service, cam_index, target_w, target_h,
                              color_space=9, fps=15, name="nao_cam_force"):
    """
    Essaie plusieurs IDs de résolution jusqu’à obtenir la taille demandée (w×h).
    Retourne l'ID d’abonnement (string) ou None si échec.
    """
    # Nettoyer un éventuel abonnement résiduel
    try:
        video_service.unsubscribe(name)
    except Exception:
        pass
    # Forcer la caméra (0=haut, 1=menton)
    try:
        video_service.setActiveCamera(cam_index)
    except Exception:
        pass

    # Candidats usuels (les IDs varient suivant le firmware)
    candidate_res_ids = [1, 2, 0, 3, 4, 5]

    for rid in candidate_res_ids:
        try:
            sub = video_service.subscribeCamera(name, cam_index, rid, color_space, fps)
        except Exception:
            sub = None
        if not sub:
            continue

        ok = False
        # Vérifier 1–2 trames pour confirmer la vraie taille
        for _ in range(2):
            img = video_service.getImageRemote(sub)
            if img is None:
                continue
            w, h = img[0], img[1]
            if w == target_w and h == target_h:
                ok = True
                break

        if ok:
            print(f"✅ Caméra configurée : {w}x{h}, cs={color_space}, res_id={rid}")
            return sub
        else:
            try:
                video_service.unsubscribe(sub)
            except Exception:
                pass

    print("❌ Impossible d’obtenir la résolution demandée.")
    return None


# --- Test caméra (aperçu rapide en couleur) ---
def camera_test_fast_color(video_service, cam_index=1, fps_req=15):
    """
    Aperçu couleur fluide (160x120), HUD retiré de l’image.
    Toutes les infos (FPS / résolution / colorspace) s'impriment en Terminal.
    ESC/q pour sortir.
    """
    cv2.setNumThreads(1)
    WINDOW = "Camera_Fast_Color"
    name = "nao_cam_fast_color"

    # Essayer YUV422(9) puis BGR(13)
    for cs in (9, 13):
        sub = subscribe_with_target_res(
            video_service, cam_index,
            target_w=160, target_h=120,
            color_space=cs, fps=fps_req, name=name
        )
        if sub:
            chosen_cs = cs
            break
    else:
        print("❌ Abonnement caméra impossible.")
        return

    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    time.sleep(0.3)

    t0 = time.time()
    frames = 0
    fps = 0

    print("\n=== 🎥 Test caméra : Terminal output (aucun texte sur image) ===")
    print(" - q/ESC : quitter\n")

    try:
        while True:
            img = video_service.getImageRemote(sub)
            if img is None:
                continue

            w, h, cs, buf = img[0], img[1], img[3], img[6]

            # Conversion image
            if cs == 9:      # YUV422 -> BGR
                arr = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 2))
                frame = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUY2)
            elif cs == 13:   # BGR direct
                frame = np.frombuffer(buf, dtype=np.uint8).reshape((h, w, 3))
            else:
                continue

            cv2.imshow(WINDOW, frame)

            # FPS measurement
            frames += 1
            dt = time.time() - t0
            if dt >= 1.0:
                fps = frames / dt
                print(f"[INFO] {w}x{h}  cs={cs}  ~{fps:.1f} FPS")
                frames = 0
                t0 = time.time()

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                break

    finally:
        try: video_service.unsubscribe(name)
        except: pass
        cv2.destroyAllWindows()
        print("✅ Caméra libérée")


# --- Diagnostic caméra (probe) ---
def camera_probe(video_service):
    """
    Teste rapidement différentes combinaisons (cam/top-bottom, résolutions, color spaces),
    et affiche si des données valides sont renvoyées.
    """
    cam_names = {0: "TOP(0)", 1: "BOTTOM(1)"}
    resolutions = {1: "160x120", 2: "320x240"}
    cspaces = {13: "BGR(13)", 11: "RGB(11)", 9: "YUV422(9)"}

    try:
        video_service.unsubscribe("probe_sub")
    except Exception:
        pass

    for cam in (1, 0):
        try:
            video_service.setActiveCamera(cam)
        except Exception:
            pass
        print("\n=== Camera:", cam_names.get(cam, cam), "===")

        for res in (2, 1):
            for req_cs in (13, 11, 9):
                try:
                    video_service.unsubscribe("probe_sub")
                except Exception:
                    pass

                try:
                    sub = video_service.subscribeCamera("probe_sub", cam, res, req_cs, 10)
                except Exception as e:
                    print(f"[SUB FAIL] cam={cam_names[cam]} res={resolutions.get(res,res)} "
                          f"req_cs={cspaces.get(req_cs,req_cs)} err={e}")
                    continue

                status = "BLACK/EMPTY"
                mean_val = None
                buflen = 0
                ret_cs = None
                layers = None
                w = h = 0
                try:
                    for _ in range(3):
                        img = video_service.getImageRemote(sub)
                        if img is None:
                            time.sleep(0.05); continue
                        w, h = img[0], img[1]
                        layers = img[2]
                        ret_cs = img[3]
                        buf = img[6] if len(img) > 6 else None
                        buflen = len(buf) if buf is not None else 0
                        if buflen == 0:
                            time.sleep(0.05); continue
                        arr = np.frombuffer(buf, dtype=np.uint8)
                        mean_val = float(arr.mean()) if arr.size else None
                        status = "OK" if (mean_val is not None and mean_val > 0) else "BLACK/EMPTY"
                        break
                finally:
                    try:
                        video_service.unsubscribe(sub)
                    except Exception:
                        pass

                print(f"[{status}] cam={cam_names[cam]} res={resolutions.get(res,res)} "
                      f"req_cs={cspaces.get(req_cs,req_cs)} ret_cs={ret_cs} layers={layers} "
                      f"buf={buflen} mean={mean_val}")
