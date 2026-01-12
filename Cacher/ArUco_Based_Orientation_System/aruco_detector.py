import cv2 as cv
import numpy as np
import sys

def read_camera_parameters(filepath = 'camera_parameters/intrinsic_iphone12.dat'):
    inf = open(filepath, 'r')
    cmtx = []
    dist = []

    #ignore first line
    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    #ignore line that says "distortion"
    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    #cmtx = camera matrix, dist = distortion parameters
    return np.array(cmtx), np.array(dist)

def get_aruco_coords(cmtx, dist, points):
    marker_size = 7.8
    #Selected coordinate points for each corner of ArUco code.
    obj_points = np.array([[0, 0, 0],
                           [marker_size, 0, 0],
                           [marker_size, marker_size, 0],
                           [0, marker_size, 0]], dtype='float32').reshape((4, 1, 3))

    #determine the orientation of ArUco coordinate system with respect to camera coorindate system.
    ret, rvec, tvec = cv.solvePnP(obj_points, points, cmtx, dist)

    #Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    axis_len = marker_size
    unitv_points = np.array([[0,0,0], [axis_len,0,0], [0,axis_len,0], [0,0,axis_len]], dtype = 'float32').reshape((4,1,3))
    
    if ret:
        points, jac = cv.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        return points, rvec, tvec
    else: 
        return [], [], []

def show_rejected_markers(img, rejected):
    # Si la liste n'est pas vide, on dessine les contours
    if rejected is not None and len(rejected) > 0:
        # borderColor=(255, 0, 255) donne du Magenta (Rose/Violet)
        cv.aruco.drawDetectedMarkers(img, rejected, borderColor=(255, 0, 255))

def read_source(cmtx, dist, in_source, show_axes=False, show_rejected=False, show_borders=False, print_coords=False):
    cap = cv.VideoCapture(in_source)

    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
    parameters = cv.aruco.DetectorParameters()

    while True:
        ret, img = cap.read()
        if ret == False: break

        # Conversion en gris (recommandé pour ArUco)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Détection des marqueurs
        corners, ids, rejected = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #Used for debugging
        if show_rejected:
            show_rejected_markers(img, rejected)

        if show_borders:
            cv.aruco.drawDetectedMarkers(img, corners, ids)

        if show_axes:
            # MODIFICATION : On vérifie juste qu'il y en a au moins un
            if ids is not None and len(ids) > 0:
                
                # On prend directement l'index 0 (le premier trouvé)
                # Plus de boucle 'for' qui parcourt tout
                current_corners = corners[0] 
                current_id = ids[0][0]

                axis_points, rvec, tvec = get_aruco_coords(cmtx, dist, current_corners)

                # Affichage des coordonnées dans la console
                if print_coords:
                    print(f"Marqueur ID {current_id} -> X: {tvec[0][0]:.2f} | Y: {tvec[1][0]:.2f} | Z: {tvec[2][0]:.2f}")

                #BGR color format
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]

                #check axes points are projected to camera view.
                if len(axis_points) > 0:
                    axis_points = axis_points.reshape((4,2))

                    origin = (int(axis_points[0][0]),int(axis_points[0][1]) )

                    for p, c in zip(axis_points[1:], colors[:3]):
                        p = (int(p[0]), int(p[1]))

                        #Sometimes detector will make a mistake and projected point will overflow integer value. We skip these cases. 
                        if origin[0] > 5*img.shape[1] or origin[1] > 5*img.shape[1]:break
                        if p[0] > 5*img.shape[1] or p[1] > 5*img.shape[1]:break

                        cv.line(img, origin, p, c, 5)

        cv.imshow('frame', img)

        k = cv.waitKey(20)
        if k == 27: break #27 is ESC key.

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':

    #read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    # Remis comme dans ton code original
    input_source = 'media/Iphone12_4.mp4'
    if len(sys.argv) > 1:
        input_source = int(sys.argv[1])

    read_source(cmtx, dist, input_source, show_borders=True)