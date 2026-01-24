# robot_wifi
# IP Address: 192.168.4.2
# Subnet: 255.255.255.0
# Gateway: 192.168.4.1


import csv,os,sys
from datetime import datetime
from flask import Flask,request

os.chdir(os.path.dirname(os.path.abspath(__file__)))

app=Flask(__name__)

if not os.path.exists("robot_data.csv"):
    with open("robot_data.csv",'w',newline="") as f:
        writer=csv.writer(f)
        writer.writerow(["time","obj_x","obj_y","obj_z","mov_x","mov_y"])


@app.route("/",methods=["POST"])
def save_robot_data():
    data=request.get_json()
    time_now=datetime.now().strftime("%H:%M:%S")
    obj_x=data["obj_x"]
    obj_y=data["obj_y"]
    obj_z=data["obj_z"]
    mov_x=data["mov_x"]
    mov_y=data["mov_y"]
    
    with open("robot_data.csv","a",newline="") as f:
        writer=csv.writer(f)
        writer.writerow([time_now,obj_x,obj_y,obj_z,mov_x,mov_y])
        
    return 0



if __name__=="__main__":
    app.run(debug=True,host= "0.0.0.0",port=5000)