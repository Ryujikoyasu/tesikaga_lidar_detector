#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tesikaga_lidar_detector.srv import TriggerCalibration
import sys

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')
        self.client = self.create_client(TriggerCalibration, '/tesikaga_calibrator/trigger_calibration')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibration service not available, waiting again...')
        self.req = TriggerCalibration.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    print("--- 屈斜路の響き - LiDARキャリブレーションシステム ---")
    print("1. 3本のペットボトルが正しく配置されていることを確認してください。")
    print("2. RViz2などで、LiDARが3つの物体のみを検出していることを確認してください。")
    
    while True:
        confirm = input("準備が完了したら 'y' を入力してキャリブレーションを実行します (nで中止): ").lower()
        if confirm == 'n':
            print("キャリブレーションを中止しました。")
            sys.exit(0)
        if confirm == 'y':
            break
        else:
            print("無効な入力です。'y' または 'n' を入力してください。")

    rclpy.init()
    calib_client = CalibrationClient()
    
    print("\nキャリブレーションサービスを呼び出しています...")
    response = calib_client.send_request()

    if response.success:
        calib_client.get_logger().info("--- キャリブレーション成功！ ---")
        print("\n" + response.message) # ノードからの詳細なメッセージを表示
    else:
        calib_client.get_logger().error("--- キャリブレーション失敗 ---")
        print("\n" + response.message)

    calib_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
