import qi
import cv2
from camera import subscribe_camera, get_frame, unsubscribe_camera

ip = "172.16.1.164"
session = qi.Session()
session.connect(f"tcp://{ip}:9559")

video_service = session.service("ALVideoDevice")

name_id = subscribe_camera(video_service, camera_index=0)
print(f"✅ Abonnement caméra ID: {name_id}")

while True:
    frame = get_frame(video_service, name_id)
    if frame is not None:
        print(f"✅ Caméra ID:")
        cv2.imshow("Caméra NAO", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

unsubscribe_camera(video_service, name_id)
cv2.destroyAllWindows()