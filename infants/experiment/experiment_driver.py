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
import rospy
import paramiko

from datetime import datetime
from pathlib import Path
from pynput import keyboard
from std_msgs.msg import String

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

NAS_PATH = Path("/home/robotlearning2/synology-tuli")  # Adjust if needed

WINDOWS_HOSTNAME = "192.168.253.101"
WINDOWS_USERNAME = "ut austin"
WINDOWS_PASSWORD = "1234"

WindowsClient = paramiko.SSHClient()
WindowsClient.set_missing_host_key_policy(paramiko.AutoAddPolicy())
WindowsClient.connect(WINDOWS_HOSTNAME, username=WINDOWS_USERNAME, password=WINDOWS_PASSWORD)

def get_ntp_offset():
    stdin, stdout, stderr = WindowsClient.exec_command('w32tm /stripchart /computer:192.168.253.201 /samples:6 /dataonly')
    output = stdout.read().decode().strip()
    values = re.findall(r'[+-]?\d+\.\d+s', output)
    offsets = [float(v.rstrip('s')) for v in values]
    mean_offset = sum(offsets[1:]) / len(offsets[1:])
    return mean_offset

def copy_subject_to_nas(subject_path: Path):
    dest_path = NAS_PATH / subject_path.name
    print(f"[INFO] Copying subject data from {subject_path} to NAS at {dest_path} ...")
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        if dest_path.exists():
            print(f"[WARN] NAS destination {dest_path} already exists, merging contents...")
        shutil.copytree(subject_path, dest_path, dirs_exist_ok=True)
        print("[INFO] Copy to NAS completed successfully.")
    except Exception as e:
        print(f"[ERROR] Failed to copy to NAS: {e}")

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


import json

class ButtonLogger:
    def __init__(self):
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.active = False

        if not rospy.core.is_initialized():
            rospy.init_node('keyboard_logger', anonymous=True)

        self.keydown_pub = rospy.Publisher('/keydown', String, queue_size=10)
        self.keyup_pub = rospy.Publisher('/keyup', String, queue_size=10)

    def _on_press(self, key):
        if not self.active:
            return
        try:
            msg = json.dumps({
                "action": "keydown",
                "key": self._key_to_str(key),
                "timestamp": datetime.now().isoformat()
            })
            print(f"[KeyDown] {msg}")
            self.keydown_pub.publish(msg)
        except Exception as e:
            print(f"[ERROR] Exception in _on_press: {e}")

    def _on_release(self, key):
        if not self.active:
            return
        try:
            msg = json.dumps({
                "action": "keyup",
                "key": self._key_to_str(key),
                "timestamp": datetime.now().isoformat()
            })
            print(f"[KeyUp] {msg}")
            self.keyup_pub.publish(msg)
        except Exception as e:
            print(f"[ERROR] Exception in _on_release: {e}")

    def _key_to_str(self, key):
        if isinstance(key, keyboard.Key):
            return key.name
        else:
            return str(key).strip("'")

    def start(self):
        self.active = True
        self.listener.start()

    def stop(self):
        self.active = False
        self.listener.stop()
        self.listener.join(timeout=2)
            

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
    # Obtain the NTP offset
    print("[INFO] Obtaining NTP offset...")
    ntp_offset = get_ntp_offset()
    metadata["ntp_offset"] = ntp_offset
    print(f"[DEBUG] NTP offset: {ntp_offset}")
    
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
	"/keydown", "/keyup",
	"-O", str(rosbag_file),
	"-q"
    ], start_new_session=True)
    print("[DEBUG] rosbag process started")

    button_logger = ButtonLogger() if use_button_log else None
    if button_logger:
        print("[DEBUG] Starting button logger...")
        button_logger.start()
        print("[DEBUG] Button logger started")

    print("[INFO] START QUALISYS NOW")
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
    rospy.init_node("experiment_driver", anonymous=True, disable_signals=True)
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
            try:
                condition = input(f"[Trial {trial_number}] Enter condition number: ").strip()
            except KeyboardInterrupt:
                print("\n[INFO] Experiment interrupted during condition input.")
                break  # Exit trials loop gracefully

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
                print("[INFO] Trial saved.\n")
            else:
                trial_path = subject_path / f"trial_{trial_number:03d}"
                if trial_path.exists():
                    shutil.rmtree(trial_path)
                    print(f"[INFO] Deleted trial folder: {trial_path}\n")
                else:
                    print(f"[WARN] Trial path not found: {trial_path}\n")

            # New input to decide whether to continue or end experiment
            cont = input("Record another condition? (y/n): ").strip().lower()
            if cont != 'y':
                print("[INFO] Ending experiment as per user request.")
                break

            trial_number += 1

    except KeyboardInterrupt:
        print("\n[INFO] Experiment ended by user.")

    # After experiment ends, copy to NAS
    # TODO: copy after each trial and not at the end!
    copy_subject_to_nas(subject_path)


if __name__ == "__main__":
    main()
