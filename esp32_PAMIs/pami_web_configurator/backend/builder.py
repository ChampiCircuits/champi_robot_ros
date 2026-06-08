import subprocess
import os

def build_and_flash():
    firmware_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'firmware')
    
    try:
        # Popen to run platformio inside the firmware path
        result = subprocess.run(["pio", "run", "--target", "upload"], cwd=firmware_dir, capture_output=True, text=True)
        return result.returncode == 0, result.stdout + result.stderr
    except Exception as e:
        return False, str(e)
