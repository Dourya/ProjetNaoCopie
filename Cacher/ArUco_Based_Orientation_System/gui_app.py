import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageTk 
import argparse

# --- IMPORTS DES MODULES ---
# Assurez-vous que les fichiers sont bien dans le dossier "moduls"
from moduls import video_processor as vp
from moduls import map_processor as mp 

class App:
    def __init__(self, root, video_source, camera_params_path):
        self.root = root
        self.root.title("Interface Contrôle Robot")
        self.root.geometry("1100x800")

        # ---------------------------------------------------------
        # 1. INITIALISATION DES PROCESSEURS (THREADS)
        # ---------------------------------------------------------
        
        # A. Processeur Vidéo
        self.video_source = video_source
        self.camera_params_path = camera_params_path
        print(f"Init Vidéo : Source='{self.video_source}'")
        self.vid_processor = vp.VideoProcessor(
            camera_params_path=self.camera_params_path, 
            source=self.video_source
        )

        # B. Processeur Carte (Game Engine / Map Logic)
        print("Init Carte...")
        self.map_processor = mp.MapProcessor()
        self.map_processor.start() # On lance le thread immédiatement

        # ---------------------------------------------------------
        # 2. MISE EN PAGE (3 COLONNES)
        # ---------------------------------------------------------
        self.root.columnconfigure(0, weight=1, uniform="group1") 
        self.root.columnconfigure(1, weight=3, uniform="group1") 
        self.root.columnconfigure(2, weight=1, uniform="group1") 
        self.root.rowconfigure(0, weight=1)

        # === GAUCHE : CONTRÔLES ===
        self.frame_left = tk.Frame(root, bg="#dddddd", padx=10, pady=10)
        self.frame_left.grid(row=0, column=0, sticky="nsew")
        
        # Section Vidéo
        tk.Label(self.frame_left, text="VIDÉO", bg="#dddddd", font=("Arial", 10, "bold")).pack(pady=(5,0))
        self.btn_start = tk.Button(self.frame_left, text="Démarrer Vidéo", command=self.toggle_video, bg="#4CAF50", fg="white", font=("Arial", 11))
        self.btn_start.pack(fill="x", pady=5)
        
        ttk.Separator(self.frame_left, orient='horizontal').pack(fill='x', pady=15)

        # Section Navigation (Les 3 étapes)
        tk.Label(self.frame_left, text="NAVIGATION", bg="#dddddd", font=("Arial", 10, "bold")).pack(pady=(5,0))
        
        self.btn_load_map = tk.Button(self.frame_left, text="1. Charger Carte", command=self.action_load_map, font=("Arial", 11))
        self.btn_load_map.pack(fill="x", pady=5)

        self.btn_find_spot = tk.Button(self.frame_left, text="2. Trouver Cachette", command=self.action_find_spot, font=("Arial", 11), state="disabled")
        self.btn_find_spot.pack(fill="x", pady=5)

        self.btn_calc_path = tk.Button(self.frame_left, text="3. Calculer Chemin", command=self.action_calc_path, font=("Arial", 11), state="disabled")
        self.btn_calc_path.pack(fill="x", pady=5)

        # === CENTRE : VISUALISATION ===
        self.frame_center = tk.Frame(root, bg="black")
        self.frame_center.grid(row=0, column=1, sticky="nsew")
        
        self.frame_center.rowconfigure(0, weight=1, uniform="rows_group") 
        self.frame_center.rowconfigure(1, weight=1, uniform="rows_group") 
        self.frame_center.columnconfigure(0, weight=1)

        # Haut : Vidéo
        self.frame_video_container = tk.Frame(self.frame_center, bg="black")
        self.frame_video_container.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        self.lbl_video = tk.Label(self.frame_video_container, bg="black", text="Caméra OFF", fg="white")
        self.lbl_video.pack(expand=True, fill="both")

        # Bas : Carte
        self.frame_map_container = tk.Frame(self.frame_center, bg="#333333") 
        self.frame_map_container.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        self.lbl_map = tk.Label(self.frame_map_container, bg="#333333", text="Aucune carte chargée", fg="#aaaaaa")
        self.lbl_map.pack(expand=True, fill="both")

        # === DROITE : INFORMATIONS ===
        self.frame_right_container = tk.Frame(root, bg="#eeeeee", padx=2, pady=2)
        self.frame_right_container.grid(row=0, column=2, sticky="nsew")
        
        tk.Label(self.frame_right_container, text="INFORMATIONS", font=("Arial", 14, "bold"), bg="#eeeeee").pack(pady=10)

        # Scrollbar setup
        self.canvas = tk.Canvas(self.frame_right_container, bg="#eeeeee", highlightthickness=0)
        self.scrollbar = tk.Scrollbar(self.frame_right_container, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg="#eeeeee")
        self.scrollable_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw", width=280)
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        self.active_markers = {} 
        self.lbl_no_data = tk.Label(self.scrollable_frame, text="Aucun robot détecté", bg="#eeeeee", fg="gray", font=("Arial", 10, "italic"))
        self.lbl_no_data.pack(pady=20)

        # ---------------------------------------------------------
        # 3. LANCEMENT DES BOUCLES DE MISE À JOUR
        # ---------------------------------------------------------
        self.is_video_playing = False
        
        # Boucle Vidéo (30ms)
        self.update_video_gui()
        
        # Boucle Carte (100ms - moins fréquent car plus lourd)
        self.update_map_gui()

    # ==========================================================
    # LOGIQUE CARTE (THREAD MAP_PROCESSOR)
    # ==========================================================

    def action_load_map(self):
        """1. Charger la carte"""
        file_path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if file_path:
            self.map_processor.send_command("LOAD_MAP", file_path)
            self.lbl_map.config(text="Chargement...")
            # On active l'étape suivante
            self.btn_find_spot.config(state="normal")

    def action_find_spot(self):
        """2. Trouver une cachette"""
        self.map_processor.send_command("FIND_HIDING_SPOT")
        # On active l'étape suivante
        self.btn_calc_path.config(state="normal")

    def action_calc_path(self):
        """3. Calculer le chemin"""
        # Idéalement, on enverrait ici la position réelle du robot détecté par ArUco
        # Pour l'instant, le MapProcessor utilise une position par défaut (5,5)
        # ou celle qu'on lui a envoyé précédemment.
        self.map_processor.send_command("CALCULATE_PATH")

    def update_map_gui(self):
        """Récupère l'image générée par le thread MapProcessor"""
        try:
            # On vide la file pour récupérer la toute dernière image disponible
            last_img = None
            while not self.map_processor.result_queue.empty():
                last_img = self.map_processor.result_queue.get_nowait()
            
            if last_img is not None:
                self.display_map_image(last_img)
        except Exception:
            pass
        
        # Rappel dans 100ms
        self.root.after(100, self.update_map_gui)

    def display_map_image(self, cv_img):
        """Affiche l'image Matplotlib dans le panel du bas"""
        panel_w = self.frame_map_container.winfo_width()
        panel_h = self.frame_map_container.winfo_height()
        if panel_w < 10: return 

        pil_image = Image.fromarray(cv_img)
        
        # Redimensionnement intelligent (Ratio)
        img_w, img_h = pil_image.size
        ratio = min(panel_w/img_w, panel_h/img_h)
        new_w = int(img_w * ratio)
        new_h = int(img_h * ratio)
        
        pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
        self.photo_map = ImageTk.PhotoImage(pil_image)
        self.lbl_map.config(image=self.photo_map, text="")

    # ==========================================================
    # LOGIQUE VIDÉO (THREAD VIDEO_PROCESSOR)
    # ==========================================================

    def toggle_video(self):
        if not self.is_video_playing:
            self.vid_processor.start()
            self.btn_start.config(text="Arrêter Vidéo", bg="#f44336")
            self.is_video_playing = True
        else:
            self.vid_processor.stop()
            self.btn_start.config(text="Démarrer Vidéo", bg="#4CAF50")
            self.lbl_video.config(image='', text="Arrêté")
            self.is_video_playing = False
            self.clear_all_markers()

    def update_video_gui(self):
        if self.is_video_playing and not self.vid_processor.data_queue.empty():
            # Récupération des données vidéo
            img_rgb, markers_data = self.vid_processor.data_queue.get()
            
            # 1. Affichage Vidéo
            self.display_video_image(img_rgb)
            
            # 2. Affichage Panel Droite (Texte)
            self.update_markers_display(markers_data)
            
            # --- MODIFICATION ICI ---
            # 3. Envoi des positions au processeur de carte
            if markers_data:
                # On envoie la liste brute. Le map_processor fera le tri par ID.
                self.map_processor.send_command("UPDATE_ROBOT_POS", markers_data)
            # ------------------------

        self.root.after(30, self.update_video_gui)

    def display_video_image(self, cv_img):
        panel_w = self.frame_video_container.winfo_width()
        panel_h = self.frame_video_container.winfo_height()
        if panel_w < 10: return 

        pil_image = Image.fromarray(cv_img)
        img_w, img_h = pil_image.size
        ratio = min(panel_w/img_w, panel_h/img_h)
        new_w = int(img_w * ratio)
        new_h = int(img_h * ratio)
        
        pil_image = pil_image.resize((new_w, new_h), Image.Resampling.LANCZOS)
        self.photo_vid = ImageTk.PhotoImage(pil_image)
        self.lbl_video.config(image=self.photo_vid, text="")

    # ==========================================================
    # GESTION DES MARQUEURS (DROITE)
    # ==========================================================

    def clear_all_markers(self):
        for mid in list(self.active_markers.keys()):
            self.active_markers[mid]['frame'].destroy()
        self.active_markers.clear()
        self.lbl_no_data.pack(pady=20)

    def update_markers_display(self, markers_data):
        incoming_ids = set()
        if markers_data:
            incoming_ids = {d['id'] for d in markers_data}
        
        current_ids = set(self.active_markers.keys())
        ids_to_remove = current_ids - incoming_ids
        
        for mid in ids_to_remove:
            self.active_markers[mid]['frame'].destroy()
            del self.active_markers[mid]
            
        if not incoming_ids:
            self.lbl_no_data.pack(pady=20)
            return
        else:
            self.lbl_no_data.pack_forget()

        for data in markers_data:
            mid = data['id']
            if mid in self.active_markers:
                widgets = self.active_markers[mid]
                widgets['lbl_x'].config(text=f"X: {data['x']:.1f}")
                widgets['lbl_y'].config(text=f"Y: {data['y']:.1f}")
                widgets['lbl_z'].config(text=f"Z: {data['z']:.1f}")
                widgets['lbl_angle'].config(text=f"Angle: {data['angle']:.0f}°")
            else:
                self.create_marker_widget(data)
                
        self.canvas.itemconfigure(self.canvas.find_all()[0], width=self.canvas.winfo_width())

    def create_marker_widget(self, data):
        mid = data['id']
        card = tk.Frame(self.scrollable_frame, bg="white", relief="raised", bd=2, padx=5, pady=5)
        card.pack(fill="x", pady=5, padx=5)

        tk.Label(card, text=f"ROBOT ID: {mid}", font=("Arial", 11, "bold"), fg="#2196F3", bg="white").pack(anchor="w")
        
        l_angle = tk.Label(card, text=f"Angle: {data['angle']:.0f}°", font=("Consolas", 10, "bold"), fg="#E91E63", bg="white")
        l_angle.pack(anchor="w")

        l_x = tk.Label(card, text=f"X: {data['x']:.1f}", font=("Consolas", 10), bg="white")
        l_x.pack(anchor="w")
        l_y = tk.Label(card, text=f"Y: {data['y']:.1f}", font=("Consolas", 10), bg="white")
        l_y.pack(anchor="w")
        l_z = tk.Label(card, text=f"Z: {data['z']:.1f}", font=("Consolas", 10), bg="white")
        l_z.pack(anchor="w")

        self.active_markers[mid] = {
            'frame': card, 
            'lbl_angle': l_angle, 
            'lbl_x': l_x, 
            'lbl_y': l_y, 
            'lbl_z': l_z 
        }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interface de contrôle robot ArUco")
    
    parser.add_argument("--source", default="0", 
                        help="Source vidéo : Chemin fichier OU numéro webcam (ex: 0)")
    
    parser.add_argument("--params", default="camera_parameters/intrinsic_iphone12.dat",
                        help="Chemin vers le fichier intrinsic.dat")

    args = parser.parse_args()

    try:
        source_input = int(args.source)
    except ValueError:
        source_input = args.source
    
    root = tk.Tk()
    app = App(root, video_source=source_input, camera_params_path=args.params)
    root.mainloop()