These files is to transcribe voice to text.Record the voice and upload to cloud, then transcribe

1.First,users are recommended to set up developing enviornment.
google cloud storage         pip install --upgrade google-cloud-storage
Cloud Speech API             pip install --upgrade google-cloud-speech
pyaudio                      pip install pyaudio
oauth2client                 https://github.com/googleapis/oauth2client
...

2.Setting up authentication(json)
 Download Google's json as Google API credentials, and run json before running the program.
 
3.type the information in program.And create a bucket in google cloud storage.

4.Example usage:
 python recordupload.py;python s2t.py gs://bucket name/.../...
 
 
