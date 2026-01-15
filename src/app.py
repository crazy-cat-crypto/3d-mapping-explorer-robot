# robot_wifi
# IP Address: 192.168.4.2
# Subnet: 255.255.255.0
# Gateway: 192.168.4.1



from flask import Flask

app=Flask(__name__)





if __name__=="__main__":
    app.run(debug=True)