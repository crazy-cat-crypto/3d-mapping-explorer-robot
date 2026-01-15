# robot_wifi
# IP Address: 192.168.4.2
# Subnet: 255.255.255.0
# Gateway: 192.168.4.1


import csv,os
from datetime import datetime
from flask import Flask,request

app=Flask(__name__)

if not os.path.exists("robot_data.csv"):
    with open("robot_data.csv",'w',newline="") as f:
        writer=csv.writer(f)
        writer.writerow(["time","distance"])


@app.route("/",methods=["POST"])
def save_robot_data():
    data=request.get_json()
    time_now=datetime.now().strftime("%H:%M:%S")
    distance=data["distance"]
    
    with open("robot_data.csv","a",newline="") as f:
        writer=csv.writer(f)
        writer.writerow([time_now,distance])
        
    return 0



if __name__=="__main__":
    app.run(debug=True,host= "0.0.0.0",port=5000)