# 🎙️🤖 Voice-Controlled Vision-Guided Robotic Arm

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.9+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python">
  <img src="https://img.shields.io/badge/Gemini_AI-Robotics-8E75B2?style=for-the-badge&logo=google&logoColor=white" alt="Gemini">
  <img src="https://img.shields.io/badge/Hardware-Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino">
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge" alt="License">
</p>

---

## 📖 Overview

A real-time perception–action loop that allows you to control a robotic arm using **natural voice commands** and **live computer vision**. This system integrates speech-to-text, vision-language modeling (Gemini), and low-level serial control.

---

## ✨ Key Features

- 🎙️ **Push-to-Talk:** Record commands by holding the `SPACE` bar
- 🧠 **Gemini Reasoning:** Multi-step task planning using vision + logic
- 👁️ **Live Vision:** Compatible with ESP32-CAM, Android IP Webcam, or CCTV
- 🗣️ **Voice Feedback:** Real-time spoken updates on what the robot is thinking
- 🔁 **Stateful Loop:** Remembers previous actions for complex, multi-stage tasks

---

## 🚀 Quick Start

### 1. Installation

```bash
git clone https://github.com/yourusername/robotic-arm-vision.git
cd robotic-arm-vision
pip install -r requirements.txt
```

### 2. Configure Environment

Create a `.env` file in the root directory:

```env
ASSEMBLYAI_API_KEY=your_key_here
GOOGLE_API=your_key_here
```

### 3. Launch the System

Run the specific script for your operating system:

| Platform | Execution Command |
|----------|-------------------|
| 🪟 **Windows** | `.\run.bat` |
| 🐧 **Linux** | `sh run.sh` |
| 🍎 **macOS** | `sh run_mac.sh` |

---

## 🛠️ System Architecture

> [!TIP]
> Each iteration of the loop is one reasoning step. The robot "sees," "thinks," "speaks," and then "moves."

1. **Input:** Voice (AssemblyAI) + Vision (Live Camera Frame)
2. **Analysis:** Gemini 1.5 Pro analyzes the scene and command
3. **Planning:** AI generates text-to-speech feedback and serial commands
4. **Execution:** Arduino moves the servos based on `MOVE` or `GRIP` strings

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐     ┌──────────────┐
│   Voice     │────>│   Gemini     │────>│  TTS + UI   │────>│   Arduino    │
│  Command    │     │   Vision AI  │     │  Feedback   │     │   Control    │
└─────────────┘     └──────────────┘     └─────────────┘     └──────────────┘
       │                    │
       └────────────┬───────┘
                    │
              ┌─────▼──────┐
              │   Camera   │
              │   Stream   │
              └────────────┘
```

---

## 💻 Platform Requirements

| OS | Extra Steps |
|----|-------------|
| **Linux** | `sudo apt install portaudio19-dev` |
| **macOS** | `brew install portaudio` |
| **Windows** | None (Works out of the box) |

---

## 🤖 Robot Command Format

The system communicates with the Arduino using a simple string protocol:

```plaintext
MOVE x y z          # Move to coordinates
GRIP OPEN           # Open the claw
GRIP CLOSE          # Close the claw
```

### Example Commands

```python
# Move arm to position
"MOVE 100 50 75"

# Pick up object
"GRIP CLOSE"

# Release object
"GRIP OPEN"
```

---

## 📦 Hardware Requirements

- Arduino board (Uno, Mega, or compatible)
- 4-6 DOF robotic arm with servo motors
- Camera (ESP32-CAM, IP Webcam, or USB webcam)
- USB cable for Arduino connection
- Power supply for servos (5V or as required)

---

## 🔧 Configuration

### Camera Setup

Edit the camera stream URL in your configuration file:

```python
#This uses an android phone with IPwebcam
CAMERA_URL = "http://192.168.1.100:81/shot"
```

### Serial Port Configuration

```python
# Windows
SERIAL_PORT = "COM3"

# Linux/macOS
SERIAL_PORT = "/dev/ttyUSB0"
```

---

## 🎯 Usage Examples

### Basic Pick and Place

1. Hold `SPACE` and say: "Pick up the red cube"
2. Robot analyzes the scene and moves to object
3. Hold `SPACE` and say: "Place it on the blue plate"
4. Robot completes the task

### Multi-Step Task

1. "Sort the blocks by color"
2. Robot plans and executes multiple pick-and-place operations
3. Provides voice feedback for each step

---

## 🛡️ Safety & Disclaimer

> [!WARNING]
> This project controls physical hardware. Please observe the following safety guidelines:

- ⚠️ Keep the robot's workspace clear of obstacles
- ⚠️ Ensure your Arduino has safety limits to prevent servo damage
- ⚠️ Always supervise the robot during operation
- ⚠️ Test in a controlled environment first
- ⚠️ Implement emergency stop functionality

---

## 🐛 Troubleshooting

### Common Issues

**Camera not connecting:**
```bash
# Verify camera URL is accessible
curl http://your-camera-ip:port/stream
```

**Audio input not working:**
```bash
# Test microphone
python -m sounddevice
```

**Arduino not responding:**
- Check serial port permissions (Linux/macOS): `sudo chmod 666 /dev/ttyUSB0`
- Verify baud rate matches Arduino sketch
- Ensure correct port is selected

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Gemini AI** - For powerful vision-language reasoning
- **AssemblyAI** - For accurate speech-to-text transcription
- **Arduino Community** - For extensive servo control resources

---

<p align="center">
  <strong>Built with ❤️ using Gemini Robotics & AssemblyAI</strong>
</p>

<p align="center">
  <a href="https://github.com/yourusername/robotic-arm-vision">⭐ Star this repo if you find it useful!</a>
</p>