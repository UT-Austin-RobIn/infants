import os
import subprocess
import time
import yaml
import csv
import threading
from datetime import datetime

from pynput import keyboard

DATA_ROOT = "data"
AUDIO_RATE = 44100


class SpacebarLogger:
    def __init__(self, trial_path):
        self.trial_path = trial_path
        self.presses = []
        self.listener = keyboard.Listener(on_press=self._on_press)
        self.lock = threading.Lock()

    def _on_press(self, key):
        if key == keyboard.Key.space:
            with self.lock:
                timestamp = datetime.now().isoformat()
                print(f"[Spacebar] Press at {timestamp}")
                self.presses.append({"timestamp": timestamp})

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()
        self.listener.join()

    def save_to_csv(self):
        if not self.presses:
            return
        filepath = os.path.join(self.trial_path, "button_events.csv")
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["timestamp"])
            writer.writeheader()
            writer.writerows(self.presses)


def make_subject_dir(subject_id):
    subject_path = os.path.join(DATA_ROOT, subject_id)
    os.makedirs(subject_path, exist_ok=True)
    return subject_path


def run_trial(subject_path, trial_number, condition_name, use_button_log=False):
    trial_name = f"trial_{trial_number:03d}"
    trial_path = os.path.join(subject_path, trial_name)
    os.makedirs(trial_path, exist_ok=True)

    rosbag_file = os.path.join(trial_path, "trial_ros")
    audio_file = os.path.join(trial_path, "trial_audio.wav")
    metadata_file = os.path.join(trial_path, "trial_metadata.yaml")

    metadata = {
        "trial_name": trial_name,
        "condition": condition_name,
        "start_time": None,
        "end_time": None
    }

    input("Press ENTER to START trial...")
    metadata["start_time"] = datetime.now().isoformat()

    # Start rosbag (limited topics)
    rosbag_proc = subprocess.Popen([
        "rosbag", "record",
        "/cam_L/color/image_raw",
        "/cam_L/aligned_depth_to_color/image_raw",
        "/cam_L/color/camera_info",
        "/cam_R/color/image_raw",
        "/cam_R/aligned_depth_to_color/image_raw",
        "/cam_R/color/camera_info",
        "/tf", "/tf_static", "/clock",
        "-O", rosbag_file,
        "-q"
    ])

    # Start audio recording
    arecord_proc = subprocess.Popen([
        "arecord", "-f", "cd", "-r", str(AUDIO_RATE), audio_file
    ])

    # Start spacebar logger
    spacebar_logger = SpacebarLogger(trial_path) if use_button_log else None
    if spacebar_logger:
        spacebar_logger.start()

    input("Press ENTER to STOP trial...")

    metadata["end_time"] = datetime.now().isoformat()

    rosbag_proc.terminate()
    arecord_proc.terminate()
    time.sleep(1)

    if spacebar_logger:
        spacebar_logger.stop()
        spacebar_logger.save_to_csv()

    with open(metadata_file, "w") as f:
        yaml.dump(metadata, f)

    return metadata


def append_to_csv(subject_path, metadata_dict):
    csv_path = os.path.join(subject_path, "metadata.csv")
    is_new = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=["trial_name", "condition", "start_time", "end_time"])
        if is_new:
            writer.writeheader()
        writer.writerow(metadata_dict)


def main():
    subject_id = input("Enter Subject ID: ").strip()
    subject_path = make_subject_dir(subject_id)

    use_button = input("Record spacebar presses? (y/n): ").lower().strip() == "y"

    print("\nReady to begin trials. Press Ctrl+C to exit.\n")

    trial_number = 1
    try:
        while True:
            condition = input(f"[Trial {trial_number}] Enter condition name: ").strip()
            metadata = run_trial(subject_path, trial_number, condition, use_button_log=use_button)
            append_to_csv(subject_path, metadata)
            trial_number += 1
    except KeyboardInterrupt:
        print("\nExperiment ended.")

if __name__ == "__main__":
    main()
