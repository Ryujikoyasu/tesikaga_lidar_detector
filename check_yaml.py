import yaml

file_path = '/home/ryuddi/ros2_ws/src/tesikaga_lidar_detector/config/detector_params.yaml'

try:
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    print("YAML file is valid.")
    # print(data)
except yaml.YAMLError as e:
    print(f"Error parsing YAML file: {e}")
