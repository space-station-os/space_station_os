import cv2
import os
import time


class VideoPlayer:
    def __init__(self, video_path: str, on_finished_callback=None):
        self.video_path = video_path
        self.on_finished_callback = on_finished_callback

    def play(self):
        # Wait until the file exists before starting
        while not os.path.exists(self.video_path):
            print(f"Waiting for video file to appear: {self.video_path}")
            time.sleep(0.5)

        cap = cv2.VideoCapture(self.video_path, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not cap.isOpened():
            print(f"Failed to open video: {self.video_path}")
            if self.on_finished_callback:
                self.on_finished_callback()
            return

        fps = cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 60.0  # fallback default
        delay = 1.0 / fps

        window_name = "Welcome to Space Station OS"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while cap.isOpened():
            start_time = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            cv2.imshow(window_name, frame)

            # Exit on ESC
            if cv2.waitKey(1) & 0xFF == 27:
                break

            elapsed = time.time() - start_time
            sleep_time = max(0, delay - elapsed)
            time.sleep(sleep_time)

        cap.release()
        cv2.destroyAllWindows()

        if self.on_finished_callback:
            self.on_finished_callback()


# ---- Auto-Play Example ----
if __name__ == "__main__":
    # Dynamically locate the video under the workspace or assets directory
    workspace = os.path.dirname(os.path.abspath(__file__))
    possible_paths = [
        os.path.join(workspace, "assets", "Ssos_begin.mp4"),
        os.path.join(workspace, "videos", "Ssos_begin.mp4"),
        os.path.expanduser("~/Videos/Ssos_begin.mp4"),
    ]

    # Pick the first path that exists
    video_path = next((p for p in possible_paths if os.path.exists(p)), None)
    if not video_path:
        print("Video not found in default locations, waiting for file...")
        video_path = possible_paths[0]  # fallback — will wait for it to appear

    def finished():
        print("Video playback completed.")

    player = VideoPlayer(video_path, on_finished_callback=finished)
    player.play()
