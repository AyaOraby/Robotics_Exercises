#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import assemblyai as aai
import os

class AudioFileProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('audio_file_processor', anonymous=True)
        
        # Set your AssemblyAI API key
        aai.settings.api_key = "7ce60176335c40b79ae58c148dfff22a"
        self.transcriber = aai.Transcriber()
        
        # Publisher for recognized speech
        self.speech_pub = rospy.Publisher('/recognized_speech_ar', String, queue_size=10)
        
        # File paths (relative to the script location)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.audio_file = os.path.join(script_dir, "../data/112.mp3")
        self.output_file = os.path.join(script_dir, "../data/112_transcription.txt")
        
        # Process the file immediately when node starts
        self.process_audio_file()

    def process_audio_file(self):
        try:
            rospy.loginfo(f"Starting processing of audio file: {self.audio_file}")
            
            # Transcribe the audio file
            transcript = self.transcriber.transcribe(
                self.audio_file,
                config=aai.TranscriptionConfig(
                    language_code="ar",
                    speech_model="nano"
                )
            )
            
            if transcript.text:
                rospy.loginfo(f"Recognized Arabic text: {transcript.text}")
                
                # Publish to ROS topic
                self.speech_pub.publish(transcript.text)
                
                # Save to file
                with open(self.output_file, "w", encoding="utf-8") as f:
                    f.write(transcript.text)
                rospy.loginfo(f"Transcription saved to: {self.output_file}")
            else:
                rospy.logwarn("No text was recognized from the audio file")
                
        except Exception as e:
            rospy.logerr(f"Error processing audio file: {str(e)}")

if __name__ == '__main__':
    try:
        processor = AudioFileProcessor()
        # Keep the node running to maintain the publisher
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Audio file processor node shutdown")

