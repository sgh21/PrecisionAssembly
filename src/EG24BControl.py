import serial
import time

class EG24BController:
    def __init__(self, port_name='COM7', baudrate=115200, timeout=1):
        self.port = serial.Serial(port_name, baudrate=baudrate, timeout=timeout)
        self.id = 0x01  # 默认ID号

    def calculate_checksum(self, data):
        """计算校验和，累加除帧头外所有字节并取低字节。"""
        return sum(data) & 0xFF

    def send_command(self, command, data=[]):
        """发送命令帧并返回响应信息。"""
        frame = [0xEB, 0x90, self.id, len(data) + 1, command] + data
        checksum = self.calculate_checksum(frame[2:])
        frame.append(checksum)
        
        # 发送命令帧
        self.port.write(bytearray(frame))
        time.sleep(0.1)  # 等待响应

        # 读取并返回应答信息
        response = self.port.read_all()
        print(f"Sent Command: {frame}")
        print(f"Response: {response.hex()}")
        return response

    def open_gripper(self, speed=500):
        """发送指令以设定速度松开夹爪到最大开口位置。"""
        print("Opening gripper...")
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF
        self.send_command(0x11, [speed_low, speed_high])

    def close_gripper(self, speed=500, force=100):
        """发送指令以设定速度和夹持力阈值夹取物体。"""
        print("Closing gripper...")
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF
        force_low = force & 0xFF
        force_high = (force >> 8) & 0xFF
        self.send_command(0x10, [speed_low, speed_high, force_low, force_high])

    def close(self):
        """关闭串口连接。"""
        self.port.close()
        print("Serial port closed.")

# 使用示例
if __name__ == '__main__':
    controller = EG24BController(port_name='COM7')
    controller.open_gripper(speed=500)   # 打开夹爪
    time.sleep(2)                        # 等待2秒
    controller.close_gripper(speed=500, force=100)  # 关闭夹爪
    controller.close()                   # 关闭串口
