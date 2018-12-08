# Subscribes to Data Published to AWS Cloud

import paho.mqtt.client as mqtt
import os
import socket
import ssl

def on_connect(client, userdata, flags, rc):
    print("Connection returned result: " + str(rc) )
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("#" , 1 )

def on_message(client, userdata, msg):
    print("topic: "+msg.topic)
    print("Payload Data: "+str(msg.payload))
    yield str(msg.payload)


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
