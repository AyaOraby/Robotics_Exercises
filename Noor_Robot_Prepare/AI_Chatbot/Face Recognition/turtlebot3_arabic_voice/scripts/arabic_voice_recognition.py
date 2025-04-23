#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import assemblyai as aai
import pyaudio
import wave
import os

class ArabicVoiceRecognition:
    def __init__(self):
        rospy.init_node('arabic_voice_recognition')
        
        # Set your AssemblyAI API key
        aai.settings.api_key = "7ce60176335c40b79ae58c148dfff22a"
        self.transcriber = aai.Transcriber()
        
        # Publisher for recognized speech
        self.speech_pub = rospy.Publisher('/recognized_speech_ar', String, queue_size=10)
        
        # Audio parameters
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.CHUNK = 1024
        self.RECORD_SECONDS = 5
        self.AUDIO_FILE = os.path.join(os.path.dirname(__file__), "../data/arabic_voice.wav")
        self.TRANSCRIPT_FILE = os.path.join(os.path.dirname(__file__), "../data/arabic_transcriptions.txt")
        
    def record_audio(self):
        audio = pyaudio.PyAudio()
        
        stream = audio.open(format=self.FORMAT, channels=self.CHANNELS,
                          rate=self.RATE, input=True,
                          frames_per_buffer=self.CHUNK)
        
        rospy.loginfo("Recording Arabic audio...")
        frames = []
        
        for _ in range(0, int(self.RATE / self.CHUNK * self.RECORD_SECONDS)):
            data = stream.read(self.CHUNK)
            frames.append(data)
        
        rospy.loginfo("Finished recording")
        
        stream.stop_stream()
        stream.close()
        audio.terminate()
        
        # Save to WAV file
        wf = wave.open(self.AUDIO_FILE, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(frames))
        wf.close()
        
    def recognize_speech(self):
        while not rospy.is_shutdown():
            # Option 1: Record live audio
            self.record_audio()
            
            # Option 2: Use existing audio file (112.mp3)
            # audio_file = os.path.join(os.path.dirname(__file__), "../data/112.mp3")
            
            try:
                # Transcribe audio
                transcript = self.transcriber.transcribe(
                    self.AUDIO_FILE,  # or audio_file for existing file
                    config=aai.TranscriptionConfig(
                        language_code="ar",
                        speech_model="nano"
                    )
                )
                
                if transcript.text:
                    rospy.loginfo(f"Recognized Arabic: {transcript.text}")
                    self.speech_pub.publish(transcript.text)
                    
                    # Save to file with UTF-8 encoding
                    with open(self.TRANSCRIPT_FILE, "a", encoding="utf-8") as f:
                        f.write(transcript.text + "\n")
                
            except Exception as e:
                rospy.logerr(f"Error in Arabic speech recognition: {str(e)}")
                
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        avr = ArabicVoiceRecognition()
        avr.recognize_speech()
    except rospy.ROSInterruptException:
        pass
