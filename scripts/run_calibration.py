#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger
import sys

class CalibrationController(Node):
    def __init__(self):
        super().__init__('calibration_controller')
        self.bg_client = self.create_client(Empty, '/tesikaga_detector/capture_background')
        self.add_point_client = self.create_client(Trigger, '/tesikaga_calibrator/add_calibration_point')
        self.calculate_client = self.create_client(Trigger, '/tesikaga_calibrator/calculate_transform')
        self.clear_client = self.create_client(Trigger, '/tesikaga_calibrator/clear_calibration')

        self.wait_for_services()

    def wait_for_services(self):
        self.get_logger().info("Waiting for calibration services...")
        if not self.bg_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Background capture service not available. Is detector_node running?")
            sys.exit(1)
        if not self.add_point_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Add point service not available. Is calibration_node running?")
            sys.exit(1)
        self.get_logger().info("All services are available.")

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

def main():
    rclpy.init()
    controller = CalibrationController()

    print("\n--- 屈斜路の響き - 新・対話型キャリブレーションシステム ---")

    # Step 1: 背景キャプチャ
    print("\n[Step 1/3] 背景のキャプチャ")
    print("エリア内に人間や障害物がない状態にしてください。")
    if input("準備ができたら 'y' を入力して背景キャプチャを開始します: ").lower() != 'y':
        print("中止しました。")
        sys.exit(0)
    
    print("背景をキャプチャ中です...")
    controller.call_service(controller.bg_client, Empty.Request())
    print("背景のキャプチャが完了しました。")

    # Step 2: 3点の登録
    print("\n[Step 2/3] マーカー位置の登録")
    points_registered = 0
    while points_registered < 3:
        print(f"\n--- ポイント {points_registered + 1} の登録 ---")
        print(f"マーカー{points_registered + 1}の位置に一人だけ立ってください。")
        action = input(f"準備ができたら 'y' を、最初からやり直す場合は 'r' を入力: ").lower()

        if action == 'r':
            print("すべての登録済みポイントをクリアします。")
            controller.call_service(controller.clear_client, Trigger.Request())
            points_registered = 0
            continue
        if action != 'y':
            print("無効な入力です。")
            continue

        response = controller.call_service(controller.add_point_client, Trigger.Request())
        if response and response.success:
            print(f"成功: {response.message}")
            points_registered += 1
        else:
            print(f"失敗: {response.message if response else 'サービス呼び出しに失敗しました。'}")
            print("もう一度試してください。")

    # Step 3: 計算
    print("\n[Step 3/3] 変換行列の計算")
    print("3点の登録が完了しました。")
    if input("変換行列を計算しますか？ 'y' を入力: ").lower() != 'y':
        print("計算を中止しました。")
        sys.exit(0)

    print("計算を実行中...")
    response = controller.call_service(controller.calculate_client, Trigger.Request())
    if response and response.success:
        print(f"\n--- キャリブレーション成功！ ---")
        print(response.message)
    else:
        print(f"\n--- キャリブレーション失敗 ---")
        print(response.message if response else 'サービス呼び出しに失敗しました。')

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
