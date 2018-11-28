from google.cloud import storage
from oauth2client.service_account import ServiceAccountCredentials
import os
import pyaudio
import wave

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 16000
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "v1.wav"

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("start talking")

frames = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("end")

stream.stop_stream()
stream.close()
p.terminate()

wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()



credentials_dict = {
    # copy the information from your json file for credential
  "type": "service_account",
  "project_id": " ",
  "private_key_id": " ",
  "private_key": "  ",
  "client_email": "  ",
  "client_id": "  ",
  "auth_uri": "  ",
  "token_uri": "  ",
  "auth_provider_x509_cert_url": "  ",
  "client_x509_cert_url": "  "
}
credentials = ServiceAccountCredentials.from_json_keyfile_dict(
    credentials_dict
)
client = storage.Client(credentials=credentials, project='myprojectname!!!!!!!!!')
bucket = client.get_bucket('mybucket location!!!!!!!!')
blob = bucket.blob('myfile location!!!!!!!!')
blob.upload_from_filename('myfile location!!!!!!!!')