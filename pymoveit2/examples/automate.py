#!/usr/bin/env python3

import subprocess
import sys
import time

def run_pick_and_place(target_color: str, scan_duration: float = 5.0):
    print(f"\n=== Starting pick-and-place for color: {target_color} ===\n")
    
    cmd = [
        "ros2", "run", "pymoveit2", "pick_and_place.py",
        "--ros-args",
        "-p", f"target_color:={target_color}",
        "-p", f"scan_duration:={scan_duration}"
    ]
    
    try:
        result = subprocess.run(cmd, check=True, text=True)
        print(f"\n=== Completed pick-and-place for color: {target_color} ===\n")
        return True
    except subprocess.CalledProcessError as e:
        print(f"\n!!! Failed for color {target_color} !!!")
        print(e)
        return False
    except Exception as e:
        print(f"\nUnexpected error for {target_color}: {e}")
        return False

def main():
    # Define the sequence you want
    sequence = ["R", "B", "G"]   # Change order or add/remove colors here
    
    print("Starting full pick-and-place sequence for all colors...\n")
    
    success_count = 0
    for color in sequence:
        if run_pick_and_place(color):
            success_count += 1
        else:
            print("Stopping sequence due to failure.")
            break
        
        # Small delay between tasks (optional but recommended)
        if color != sequence[-1]:
            print("Waiting 2 seconds before next color...\n")
            time.sleep(2)
    
    print(f"\n=== All tasks finished! {success_count}/{len(sequence)} successful ===\n")

if __name__ == "__main__":
    main()
