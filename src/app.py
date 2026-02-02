# settings setup
# robot_wifi
# IP Address: 192.168.4.2
# Subnet: 255.255.255.0
# Gateway: 192.168.4.1

import frontier
import csv,os
from flask import Flask
from flask_socketio import SocketIO,emit

os.chdir(os.path.dirname(os.path.abspath(__file__)))

app=Flask(__name__)
socketio=SocketIO(app,cors_allowed_origins="*")

if not os.path.exists("robot_data.csv"):
    with open("robot_data.csv",'w',newline="") as file:
        writer=csv.writer(file)
        writer.writerow(["time","x","y","Q","dist","pan","tilt"])


@socketio.on("connect")
def connect():
    print("Robot Connected")
    
@socketio.on("disconnect")
def disconnect():
    print("Robot disconnected")

@socketio.on("datas")
def save_robot_data(datas):
    for data in datas:
        time=data["time"]
        x=data["x"]
        y=data["y"]
        dist=data["dist"]
        pan=data["pan"]
        tilt=data["tilt"]
        Q=data["Q"]
        
        with open("robot_data.csv","a",newline="") as file:
            writer=csv.writer(file)
            writer.writerow([time,x,y,Q,dist,pan,tilt])
            
        socketio.start_background_task(waypoints)
        
    return 0

def waypoints():
    waypoint=select_frontier()
    socketio.emit("waypoint",waypoint)

if __name__=="__main__":
    socketio.run(app,debug=True,host= "0.0.0.0",port=5000)