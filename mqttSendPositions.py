import paho.mqtt.client as mqtt
import time

def on_log(client, userdata, level, buf):
    print("log: "+buf)

# Sender connects to Broker
def on_connect(client, userdata, flags, rc):
    if rc==0:
        #if connection was succesful
        print("connected Ok")
    else:
        #if connection fails
        print("bad connection returned code= ", rc)

# When user disconnects
def on_disconnect(client, userdata, flags, rc= 0):
    print("disconnected result code "+str(rc))

def sendPositions(positions):
# Broker message is being sent to
    broker = "mqtt.eclipseprojects.io"
# Client sending message
    client = mqtt.Client("JoeSender")

# Function calls
    client.on_connect = on_connect
    client.on_log = on_log
    client.on_disconnect= on_disconnect

# When connecting to broker
    print("connecting to broker", broker)
    client.connect(broker)

# Start loop to send message
    client.loop_start()
    print("=======================================")
    print("Message being sent: " + str(positions))
    print("=======================================")
# This is the message client.publish("Topic", "Message contents")
    client.publish("IGSProjectSendCoordinates", str(positions))
    time.sleep(5)
## Stops once message is sent
    client.loop_stop()
# Disconnects from Broker
    client.disconnect()