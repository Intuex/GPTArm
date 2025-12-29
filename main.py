import os
import time
import numpy as np
import sounddevice as sd
import keyboard
from scipy.io.wavfile import write
import assemblyai as aai
import pyttsx3
from dotenv import load_dotenv
from google import genai
from google.genai import types
import serial
import requests


load_dotenv()
aai.settings.api_key = os.getenv("ASSEMBLYAI_API_KEY")
GOOGLE_API_KEY = os.getenv("GOOGLE_API")

MODEL_ID = "gemini-robotics-er-1.5-preview"
client = genai.Client(api_key=GOOGLE_API_KEY)


URL = "http://192.168.1.15:8080/shot"
IMAGE_PATH = "images/input.jpg"
AUDIO_PATH = "audio/recording.wav"
SAMPLE_RATE = 44100
CHANNELS = 1


engine = pyttsx3.init()
engine.setProperty("rate", 170)

def speak(text):
    engine.say(text)
    engine.runAndWait()


def record_audio():
    audio_chunks = []
    recording = False

    def callback(indata, frames, time, status):
        if recording:
            audio_chunks.append(indata.copy())

    print("Hold SPACE to record")

    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=CHANNELS,
        dtype="int16",
        callback=callback
    ):
        while True:
            if keyboard.is_pressed("space"):
                if not recording:
                    print("Recording...")
                    recording = True
            else:
                if recording:
                    print("Stopped recording")
                    break

    audio = np.concatenate(audio_chunks, axis=0)
    write(AUDIO_PATH, SAMPLE_RATE, audio)
    print("Audio saved")


commands_history = []
count = 0

while True:
    print(f"\n===== STEP {count} =====")


    r = requests.get(URL, timeout=5)
    if r.status_code == 200:
        with open(IMAGE_PATH, "wb") as f:
            f.write(r.content)
        print("Image updated")
    else:
        print("Image download failed")
        continue

 
    record_audio()

    transcript = aai.Transcriber().transcribe(AUDIO_PATH)
    if transcript.status == "error":
        print("Transcription failed")
        continue

    base_prompt = transcript.text
    print("You said:", base_prompt)

    speak("Processing")

    
    if count == 0:
        prompt = f"""
You are a robotic arm.
User request: "{base_prompt}"

First line: explain briefly what you are doing.
Next lines: commands only:
MOVE x y z
GRIP OPEN
GRIP CLOSE
"""
    else:
        prompt = f"""
You are a robotic arm.
User request: "{base_prompt}"

Previous actions:
{commands_history}

First line: explain briefly what you are doing.
Next lines: commands only:
MOVE x y z
GRIP OPEN
GRIP CLOSE
"""

    with open(IMAGE_PATH, "rb") as f:
        image_bytes = f.read()

    response = client.models.generate_content(
    model=MODEL_ID,
    contents=[
        types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
        prompt
        ],
            config=types.GenerateContentConfig(
                temperature=0.5,
                thinking_config=types.ThinkingConfig(thinking_budget=0)
            )
    )

    raw_text = response.text or ""
    raw_text = raw_text.strip()

    print("\nMODEL RESPONSE:\n", raw_text)


    lines = [l.strip() for l in raw_text.split("\n") if l.strip()]
    spoken = []
    commands = []

    for line in lines:
        if line.startswith("MOVE") or line.startswith("GRIP"):
            commands.append(line)
        else:
            spoken.append(line)

    if spoken:
        speak(spoken[0])

    if not commands:
        print("No commands generated")
        continue


    ser = serial.Serial("COM7", 9600, timeout=2)
    time.sleep(2)

    for cmd in commands:
        print("Sending:", cmd)
        ser.write((cmd + "\n").encode())
        time.sleep(1)

    ser.close()

    commands_history.extend(commands)
    count += 1
