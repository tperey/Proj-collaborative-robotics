# need to access robot mic and interpret the verbal command 
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D
from scipy.integrate import solve_ivp
import cv2
import os
#from google.colab import files
from PIL import Image, ImageDraw
import wave
#from google.cloud import speech_v1p1beta1 as speech
from google.cloud import speech_v1 as speech
#from google.colab import files
from google.cloud import vision
from PIL import Image as PILImage, ImageDraw, ImageFont
from IPython.display import display
import string 
import re 

data_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\data\data\VLM and Audio'
json_key_path = r'C:\Users\capam\Documents\stanford\colloborative_robotics\python-447906-51258c347833.json'
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = json_key_path

class SpeechTranscriber:
    def __init__(self, language_code='en-US', sample_rate=16000):
        """
        Initialize a SpeechTranscriber instance.

        :param language_code: The language code for transcription, e.g., 'en-US'.
        :param sample_rate: The sample rate (Hertz) of the audio file, default is 16000.
        """
        self.language_code = language_code
        self.sample_rate = sample_rate
        self.client = speech.SpeechClient()

    def transcribe_audio(self, audio_content):
        """
        Uses Google Cloud Speech-to-Text to transcribe the given audio content (bytes).
        Returns the transcription as a string.

        :param audio_content: The raw bytes of the audio file to be transcribed.
        :return: A string of the combined transcription.
        """
        audio = speech.RecognitionAudio(content=audio_content)

        config = speech.RecognitionConfig(
            sample_rate_hertz=self.sample_rate,
            language_code=self.language_code,
            enable_automatic_punctuation=True,
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        )
        talk = self.client.recognize(config=config, audio=audio)
        transcript = [result.alternatives[0].transcript for result in talk.results]
        return transcript
    
test = SpeechTranscriber()

audio_content1 = data_path + "/recorded_audio1.wav"
with open(audio_content1, "rb") as audio_file:
    audio_content1 = audio_file.read()
transcription = test.transcribe_audio(audio_content1)
print("Transcription:\n", transcription)
