import socket
import numpy as np

class UdpSender:
    """指定されたIPアドレスとポートにUDPパケットを送信する責務を持つ。"""
    def __init__(self, ip: str, port: int, logger):
        self._ip = ip
        self._port = port
        self._logger = logger
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._logger.info(f"UDP sender configured for {self._ip}:{self._port}")

    def send_data(self, data: np.ndarray):
        """numpy配列をバイト列に変換して送信する。"""
        if data.size == 0:
            return
        
        try:
            # 配列をfloat32型のバイト列に変換
            payload = data.astype(np.float32).tobytes()
            self._sock.sendto(payload, (self._ip, self._port))
        except Exception as e:
            self._logger.error(f"Failed to send UDP packet: {e}")

    def close(self):
        self._sock.close()
        self._logger.info("UDP socket closed.")