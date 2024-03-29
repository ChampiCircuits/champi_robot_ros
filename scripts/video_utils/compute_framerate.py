import cv2
import time

def open_webcam_and_compute_framerate(width, height):
    cap = cv2.VideoCapture(0)

    # Set the video resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    num_frames = 10
    start_time = time.time()

    for _ in range(num_frames):
        ret, frame = cap.read()

    end_time = time.time()

    # Compute and print the frame rate
    fps = num_frames / (end_time - start_time)
    print(f"Frame rate: {fps} fps")

    cap.release()
    cv2.destroyAllWindows()

# Call the function with desired resolution
# open_webcam_and_compute_framerate(640, 480)
open_webcam_and_compute_framerate(1920, 1080)