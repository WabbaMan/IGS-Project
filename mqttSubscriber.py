import os
import paho.mqtt.client as mqtt
import time
import cv2
import ImageProcessing as imgProcess

def on_log(client, userdata, level, buf):
    print("log: "+buf)

# When a user connects to the broker
def on_connect(client, userdata, flags, rc):
    if rc==0:
        #if connection was succesful
        print("connected Ok")
    else:
        #if connection failed
        print("bad connection returned code= ", rc)

# When user disconnects print disconnect result code
def on_disconnect(client, userdata, flags, rc= 0):
    print("disconnected result code "+str(rc))

# When user recieves message
def on_message(client, userdata, msg):
    #Sets the topic the user wants to recieve
    topic=msg.topic
    #Decodes message
    m_decode=str(msg.payload.decode("utf_8","ignore"))
    #Tells user of success
    print("message recieved: ", m_decode)
    #starts image processing program
    imgProcess.startProgram()
    

# What broker the client is using
broker = "mqtt.eclipseprojects.io"
# Who the client is
client = mqtt.Client("JoeReciever")

# Function calls
client.on_connect = on_connect
client.on_log = on_log
client.on_disconnect= on_disconnect
client.on_message= on_message

# When user tries to connect to broker
print("connecting to broker", broker)
client.connect(broker)

# Starts loop that waits for message to be recieved
client.loop_start
# Subscribes to topic
client.subscribe("IGSProjectRunDetectionTest1")
time.sleep(4)
# Loops forever, recieving messages whenever broker gets them
client.loop_forever()