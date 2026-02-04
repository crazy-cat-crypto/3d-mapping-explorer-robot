import serial
import time

class SerialInterface:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.1)

    def read_packet(self):
        """
        Expected:
        time,x,y,theta,distance,angle
        """
        line = self.ser.readline().decode().strip()
        if not line:
            return None

        try:
            data = list(map(float, line.split(",")))
            return { 
                "t": data[0],
                "x": data[1],
                "y": data[2],
                "theta": data[3],
                "dist": data[4] / 100.0,  # cm → meters
                "angle": data[5]
            }
        except:
            return None

    def send_waypoint(self, x, y):
        cmd = f"{x:.2f},{y:.2f}\n"
        self.ser.write(cmd.encode())
