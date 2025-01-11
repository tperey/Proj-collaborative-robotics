import io
from google.cloud import vision
from PIL import Image as PILImage
import sounddevice as sd
import wave
from google.cloud import speech_v1p1beta1 as speech
import google.generativeai as genai


class Google_API:
    def __init__(self):
        genai.configure(api_key='AIzaSyCptM2l3SVH6ZBbsST_2m_haTIG-pCslyc')
        self.gemini_model = genai.GenerativeModel("gemini-1.5-flash")
        self.prompt = "In the given voice transcript, identify what the object is that the user wants. Return only the object in lowercase and do not include any whitespaces, punctuation, or new. Here is the voice transcript: "

    def find_center(self, image_path, object):
        # Initialize the Vision client
        client = vision.ImageAnnotatorClient()

        # Read image into memory
        with io.open(image_path, 'rb') as image_file:
            content = image_file.read()
        image = vision.Image(content=content)

        # Send the image to the API for object localization
        response = client.object_localization(image=image)

        # Extract localized object annotations
        objects = response.localized_object_annotations

        # We’ll look for the object named "Apple" (case-insensitive)
        for obj in objects:
            if obj.name.lower() == object.lower():
                # We have found an "Apple"
                bounding_poly = obj.bounding_poly

                # bounding_poly contains normalized vertices (0 to 1)
                normalized_vertices = bounding_poly.normalized_vertices

                # Calculate the average of the vertices to find the normalized center
                avg_x = sum([v.x for v in normalized_vertices]) / len(normalized_vertices)
                avg_y = sum([v.y for v in normalized_vertices]) / len(normalized_vertices)

                # Convert normalized coordinates back to pixel coordinates
                # 1) Open the image again (e.g., with Pillow) to get width & height
                
                pil_image = PILImage.open(image_path)
                width, height = pil_image.size

                # 2) Multiply normalized coords by actual dimensions
                pixel_x = int(avg_x * width)
                pixel_y = int(avg_y * height)

                print(f"Found with approximate center: ({pixel_x}, {pixel_y})")
                return (pixel_x, pixel_y)

        print("No object detected in the image.")
        return None
    
    def record_audio(self, filename, duration=5, sample_rate=16000):
        """
        Records audio from the microphone for a given duration (in seconds) and
        sample rate, then saves it to a WAV file.
        """
        print("Recording started...")
        # Record audio (mono channel)
        recorded_data = sd.rec(
            int(duration * sample_rate),
            samplerate=sample_rate,
            channels=1,
            dtype='int16'  # 16-bit PCM
        )
        sd.wait()  # Wait until recording is complete
        print("Recording complete. Saving audio...")

        # Write the audio data to a WAV file
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(1)                 # mono
            wf.setsampwidth(2)                # 2 bytes (16 bits)
            wf.setframerate(sample_rate)
            wf.writeframes(recorded_data.tobytes())
        print(f"Audio saved to {filename}")


    def transcribe_audio(self, filename, sample_rate=16000):
        """
        Uses Google Cloud Speech-to-Text to transcribe the given WAV audio file.
        Returns the transcription as a string.
        """
        client = speech.SpeechClient()

        # Read the audio file into memory
        with open(filename, 'rb') as audio_file:
            content = audio_file.read()

        audio = speech.RecognitionAudio(content=content)
        config = speech.RecognitionConfig(
            sample_rate_hertz=sample_rate,
            language_code='en-US',
            enable_automatic_punctuation=True,
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        )

        response = client.recognize(config=config, audio=audio)

        # Extract the transcription from the response
        full_transcript = []
        for result in response.results:
            # Each "result" may contain multiple alternatives
            full_transcript.append(result.alternatives[0].transcript)

        return " ".join(full_transcript)
    
    def generate_content(self, text):
        print("Generating content for:", text)
        input = self.prompt + text
        response = self.gemini_model.generate_content(input)
        return response.text
    
    def combined_api(self, image_path, user_audio_text):
        raw_item = self.generate_content(user_audio_text)
        print("Gemini raw output:", repr(raw_item)) 
        item = raw_item.strip().lower().rstrip(".!?,")
        center_coordinates = self.find_center(image_path, item)
        if center_coordinates:
            return center_coordinates
        else:
            return None
        
def main():
    api = Google_API()
    image_path = "image2.jpg"
    api.record_audio("recorded_audio.wav", duration=5, sample_rate=16000)
    user_audio_text = api.transcribe_audio("recorded_audio.wav")
    # print("Transcribed text: " + user_audio_text)
    # print(api.generate_content(user_audio_text))
    center_coordinates = api.combined_api(image_path, user_audio_text)
    print("Center in pixel coordinates:", center_coordinates)

    image_path = "image2.jpg"
    center_coordinates = api.find_center(image_path,'dog')
    print("Test: ", center_coordinates)

if __name__ == "__main__":
    main()