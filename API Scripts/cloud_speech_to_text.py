import sounddevice as sd
import wave
from google.cloud import speech_v1p1beta1 as speech

def record_audio(filename, duration=5, sample_rate=16000):
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


def transcribe_audio(filename, sample_rate=16000):
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


def main():
    # 1. Record from microphone
    filename = "recorded_audio.wav"
    record_audio(filename, duration=5, sample_rate=16000)

    # 2. Transcribe the recorded audio
    transcription = transcribe_audio(filename)
    print("Transcription:\n", transcription)


if __name__ == "__main__":
    main()
