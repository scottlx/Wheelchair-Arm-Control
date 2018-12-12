# Subscribes to Data Published to AWS Cloud

import paho.mqtt.client as mqtt
import os
import socket
import ssl
import sys
sys.path.insert(0, './../pointnet')
from inference import evaluate


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("connect to Alexa success")
    else:
        print("Connection returned result: " + str(rc) )
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("#" , 1 )

def on_message(client, userdata, msg):
    print("topic: "+msg.topic)
    print("Payload Data: "+str(msg.payload))
    if str(msg.payload) == 'bottle':
        os.chdir("./../pointnet")
        location, std = evaluate(label_to_detect=12, x_offset=0.35,y_offset=0.137, z_offset=0.1)
        print(location, std)


mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message


# Define the AWS Host Key  ; Thing Name defined in AWS IoT; Root Certificate Path; Certificate Path; Private Key Certificate Path
awshost = "a2hrbj4onpnif8-ats.iot.us-east-1.amazonaws.com"
# AWS Port(Default: 8883)
awsport = 8883
# Client ID
clientId = "ARM"
# Thing Name defined in AWS IoT
thingName = "ARM"
# Root Certificate Path
caPath = "AmazonRootCA1.pem"
# Certificate Path
certPath = "f0fdc4db00-certificate.pem.crt"
# Private Key Certificate Path
keyPath = "f0fdc4db00-private.pem.key"

mqttc.tls_set(caPath, certfile=certPath, keyfile=keyPath, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)
mqttc.connect(awshost, awsport, keepalive=60)
mqttc.loop_forever()
