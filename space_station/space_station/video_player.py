import cv2
import os
import sys


class VideoPlayer:
    def __init__(self, video_path: str, on_finished_callback=None):
        self.video_path = video_path
        self.on_finished_callback = on_finished_callback

    def play(self):
        if not os.path.exists(self.video_path):
            print(f"Video not found: {self.video_path}")
            if self.on_finished_callback:
                self.on_finished_callback()
            return

        cap = cv2.VideoCapture(self.video_path)
        if not cap.isOpened():
            print(f"Failed to open video: {self.video_path}")
            if self.on_finished_callback:
                self.on_finished_callback()
            return

        window_name = "Welcome to Space Station OS"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame = cv2.resize(frame, (640, 480))
            cv2.imshow(window_name, frame)

            if cv2.waitKey(30) & 0xFF == 27: 
                break

        cap.release()
        cv2.destroyAllWindows()

        if self.on_finished_callback:
            self.on_finished_callback()


# ---- Test block ----
if __name__ == "__main__":
    # Example: Adjust this to your video file path
    test_video_path = os.path.join(os.path.dirname(__file__), 'assets', 'Ssos_begin.mp4')

    def finished():
        print("Video playback completed.")

    player = VideoPlayer(test_video_path, on_finished_callback=finished)
    player.play()
