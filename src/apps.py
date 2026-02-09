# disable windows firewall
# set network private
# control panel setup
# robot_wifi
# IP Address: 192.168.1.213
# Subnet: 255.255.255.0,24
# Gateway: 192.168.4.1
import random as rand
import csv,os,time
from flask import Flask,request,jsonify
import threading

os.chdir(os.path.dirname(os.path.abspath(__file__)))
count=0
app=Flask(__name__)

if not os.path.exists("robot_data.csv"):
    with open("robot_data.csv",'w',newline="") as file:
        writer=csv.writer(file)
        writer.writerow(["time","x","y","Q","dist","pan","tilt"])

target_x=25
target_y=25

def aim():
    global target_x,target_y
    target_x,target_y=frontier_exploration()

@app.route("/display",methods=["POST"])
def display():
    print(request.get_json())
    return "o"


@app.route("/data",methods=["POST"])
def save_robot_data():
    global count,target_x,target_y
    count+=1
    datas=request.get_json()
    with open("robot_data.csv","a",newline="") as file:
        writer=csv.writer(file)
        for data in datas:
            writer.writerow([time.time(),data.get("x"),data.get("y"),data.get("Q"),data.get("dist"),data.get("pan"),data.get("tilt")])
    if count % 10 ==0:
        target_x=rand.randint(0,100)
        target_y=rand.randint(0,100)
    return jsonify({"tx":target_x,"ty":target_y})


if __name__=="__main__":
    app.run(debug=True,host= "0.0.0.0",port=5000)