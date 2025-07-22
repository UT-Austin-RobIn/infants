import subprocess
import time
import yaml
import csv
import shutil
import threading
import re
import sys
import termios
import tty
import select

from datetime import datetime
from pathlib import Path
from pynput import keyboard

DATA_ROOT = Path("data")
AUDIO_RATE = 44100

CONDITIONS = {
    'bang': """
    1. Soft Board - Headphones			(low haptics,  low audio )
    2. Soft Board - No Headphones 		(low haptics,  high audio)
    3. Hard Board - Headphones 			(high haptics, low audio )
    4. Hard Board - No Headphones 		(high haptics, high audio)
    5. Wash Board - Headphones 			(high haptics, low audio )
    6. Wash Board - No Headphones 		(high haptics, high audio)
    7. Soft Board and Button - Headphones 	(high haptics, low audio )
    8. Soft Board and Button - No Headphones 	(high haptics, high audio)
    """,
    'slide': """
    1. washboard-sphere         (high haptic, high audio)
    2. soft-sphere              (high haptic, low audio)
    3. soft-rattle              (low haptic, high audio)
    4. washboard-sphere-muffled (high haptic, low audio)
    """,
    'hammer': ''
}

def flush_stdin():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def get_next_trial_number(subject_path: Path):
    trial_dirs = [
        p for p in subject_path.iterdir()
        if p.is_dir() and re.match(r"trial_\d{3}", p.name)
    ]
    if not trial_dirs:
        return 1
    trial_numbers = [int(p.name.split('_')[1]) for p in trial_dirs]
    return max(trial_numbers) + 1


class ButtonLogger:
    def __init__(self, trial_path: Path):
        self.trial_path = trial_path
        self.presses = []
        self.listener = keyboard.Listener(on_press=self._on_press)
        self.active = False  # Flag to track when we're actually logging

    def _on_press(self, key):
        try:
            if not self.active:
                return  # Ignore all keypresses if not active
            if key == keyboard.Key.enter:
                timestamp = datetime.now().isoformat()
                print(f"[Button] ENTER Press at {timestamp}")
                self.presses.append({"timestamp": timestamp})
        except Exception as e:
            print(f"[ERROR] Exception in ButtonLogger _on_press: {e}")

    def start(self):
        self.active = True
        self.listener.start()

    def stop(self):
        print("[DEBUG] ButtonLogger stopping...")
        self.active = False
        self.listener.stop()
        print("[DEBUG] Waiting for listener thread to join...")
        self.listener.join(timeout=2)
        if self.listener.is_alive():
            print("[WARN] Button listener thread did not exit cleanly")
        else:
            print("[DEBUG] Listener thread exited cleanly")

    def save_to_csv(self):
        if not self.presses:
            return
        filepath = self.trial_path / "button_events.csv"
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=["timestamp"])
            writer.writeheader()
            writer.writerows(self.presses)
            

def make_subject_dir(subject_id: str) -> Path:
    subject_path = DATA_ROOT / subject_id
    subject_path.mkdir(parents=True, exist_ok=True)
    return subject_path


def wait_for_keypress(target_key):
    print(f"Waiting for {target_key.name.upper()} press to continue...")
    key_pressed = threading.Event()

    def on_press(key):
        if key == target_key:
            key_pressed.set()
            return False  # Stop listener

    with keyboard.Listener(on_press=on_press) as listener:
        key_pressed.wait()


def run_trial(subject_path: Path, trial_number: int, condition_name: str, task_name: str, use_button_log=False):
    trial_name = f"trial_{trial_number:03d}"
    trial_path = subject_path / trial_name
    trial_path.mkdir(parents=True, exist_ok=True)

    rosbag_file = trial_path / "trial_ros"
    metadata_file = trial_path / "trial_metadata.yaml"

    metadata = {
        "trial_name": trial_name,
        "condition": condition_name,
        "start_time": None,
        "end_time": None,
        "task": task_name,
    }

    print("[DEBUG] Waiting for SPACE to start trial...")
    wait_for_keypress(keyboard.Key.space)
    metadata["start_time"] = datetime.now().isoformat()
    print(f"[DEBUG] Trial started at {metadata['start_time']}")

    # Start rosbag
    print("[INFO] Starting rosbag...")
    rosbag_proc = subprocess.Popen([
        "rosbag", "record", "-b", "0",
        "/cam_L/color/image_raw",
        "/cam_L/aligned_depth_to_color/image_raw",
        "/cam_L/color/camera_info",
        "/cam_R/color/image_raw",
        "/cam_R/aligned_depth_to_color/image_raw",
        "/cam_R/color/camera_info",
        "/cam_M/color/image_raw",
        "/cam_M/aligned_depth_to_color/image_raw",
        "/cam_M/color/camera_info",
        "/tf", "/tf_static", "/clock",
        "/audio/audio",
        "-O", str(rosbag_file),
        "-q"
    ], start_new_session=True)
    print("[DEBUG] rosbag process started")

    button_logger = ButtonLogger(trial_path) if use_button_log else None
    if button_logger:
        print("[DEBUG] Starting button logger...")
        button_logger.start()
        print("[DEBUG] Button logger started")

    print("[DEBUG] Waiting for SPACE to end trial...")
    wait_for_keypress(keyboard.Key.space)
    metadata["end_time"] = datetime.now().isoformat()
    print(f"[DEBUG] Trial ended at {metadata['end_time']}")

    print("[INFO] Stopping rosbag...")
    rosbag_proc.terminate()
    try:
        rosbag_proc.wait(timeout=5)
        print("[DEBUG] rosbag terminated cleanly")
    except subprocess.TimeoutExpired:
        print("[WARN] rosbag did not terminate in time. Killing forcibly...")
        rosbag_proc.kill()
        rosbag_proc.wait()
        print("[DEBUG] rosbag process killed")

    if button_logger:
        print("[DEBUG] Stopping button logger...")
        button_logger.stop()
        print("[DEBUG] Button logger stopped")
        print("[DEBUG] Saving button events to CSV...")
        button_logger.save_to_csv()
        print("[DEBUG] Button events saved")

    print("[DEBUG] Writing trial metadata to YAML...")
    with open(metadata_file, "w") as f:
        yaml.dump(metadata, f)
    print("[DEBUG] Metadata written")

    print()  # Ensure newline before returning
    return metadata

def append_to_csv(subject_path: Path, metadata_dict):
    csv_path = subject_path / "metadata.csv"
    is_new = not csv_path.exists()
    with open(csv_path, "a", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=metadata_dict.keys())
        if is_new:
            writer.writeheader()
        writer.writerow(metadata_dict)


def main():
    subject_id = input("Enter Subject ID: ").strip()
    subject_path = make_subject_dir(subject_id)
    task = input("Enter task name [bang/slide/hammer]: ").strip()
    if task not in CONDITIONS:
    	print(f"[WARN] Unknown task '{task}', defaulting to 'bang'")
    	task = "bang"

    print("\nReady to begin trials. Press Ctrl+C to exit.\n")

    trial_number = get_next_trial_number(subject_path)
    try:
        while True:
            print('Conditions:')
            print(CONDITIONS[task])
            condition = input(f"[Trial {trial_number}] Enter condition number: ").strip()
            metadata = run_trial(
                subject_path,
                trial_number,
                condition,
                task,
                use_button_log=condition.strip() in ["7", "8"]
            )
		
            flush_stdin()
            keep = input("\nKeep trial? (y/n): ").strip().lower()
            if keep == "y":
                append_to_csv(subject_path, metadata)
                trial_number += 1
                print("[INFO] Trial saved.\n")
            else:
                trial_path = subject_path / f"trial_{trial_number:03d}"
                if trial_path.exists():
                    shutil.rmtree(trial_path)
                    print(f"[INFO] Deleted trial folder: {trial_path}\n")
                else:
                    print(f"[WARN] Trial path not found: {trial_path}\n")



    except KeyboardInterrupt:
        print("\n[INFO] Experiment ended by user.")


if __name__ == "__main__":
    main()

