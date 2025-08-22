#!/usr/bin/env python3
import os

def main():
    calib_file = '/home/ryuddi/ros2_ws/src/tesikaga_lidar_detector/config/calibration_points.json'
    
    try:
        os.remove(calib_file)
        print("✅ Calibration points cleared.")
    except FileNotFoundError:
        print("ℹ️  No calibration points file found. Already clear.")

if __name__ == '__main__':
    main()