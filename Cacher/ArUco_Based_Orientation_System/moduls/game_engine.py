class GameEngine:
    def __init__(self, camera_params_path='camera_parameters/intrinsic_iphone12.dat', source=0):
        self.source = source
        self.cmtx, self.dist = self.read_camera_parameters(camera_params_path)
        self.running = False
        self.thread = None
        
        # File d'attente pour communiquer avec l'interface graphique
        self.data_queue = queue.Queue(maxsize=1) 
        
        # Paramètres
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
        self.parameters = cv.aruco.DetectorParameters()
        self.marker_size = 7.8 # cm