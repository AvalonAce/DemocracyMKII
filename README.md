# Democracy Robot — ROS2

ROS2 Python package for my bipedal robot project.

## What's Inside
- **activate** — main control node, manages system state
- **voice_service** — queued audio playback node
- **mic_node** — wake word detection and speech recognition

## Not Included
- Wake word model (`.ppn` file) — train your own at [console.picovoice.ai](https://console.picovoice.ai)
- Audio files (`/audio/voice_lines/`)
- API keys — copy `.env.example` to `.env` and fill in your own
- All filepaths are global to my robot, so they'll need to be changed.

## Setup
```bash
cd ~/ros2_ws
colcon build --packages-select democracy_pkg
source install/setup.bash
```

## Run
```bash
./activate.sh
```

## Dependencies
```bash
pip install pvporcupine vosk SpeechRecognition pyaudio python-dotenv --break-system-packages
sudo apt install mpg123 portaudio19-dev
```

## License
MIT License — see [LICENSE](LICENSE)