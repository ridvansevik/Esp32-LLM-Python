# ESP32 & Python Smart Assistant and Gas Detection System

## Overview

This project is a smart assistant system capable of interacting via voice commands, monitoring ambient gas levels, and sending real-time notifications. The system consists of two main components:

1.  **ESP32 Microcontroller (Arduino Code):** Collects environmental sensor data (sound, gas), communicates via Wi-Fi, plays audio, and provides alerts.
2.  **Python Server:** Processes audio data from the ESP32, generates responses using cloud-based AI services (speech recognition, text-to-speech, language model), receives gas data, and generates audio alerts when necessary.

## Arduino Code (ESP32)

The `ArduinoCodeRidvanSevik-211401051.ino` file contains the C/C++ based Arduino code for the ESP32 microcontroller. Its main functions are:

* **Wi-Fi Connection:** Connects to the specified Wi-Fi network.
* **UDP Communication:**
    * Sends audio data captured from the microphone to the Python server via UDP.
    * Receives audio data (e.g., assistant responses) from the Python server via UDP.
    * Periodically sends data read from the MQ2 gas sensor to the Python server via UDP.
* **I2S Audio Management:**
    * `I2S_NUM_0`: Configured to read audio data from an external microphone.
    * `I2S_NUM_1`: Configured to play audio through an external speaker/amplifier.
* **Sensor Integration:**
    * **MQ2 Gas Sensor:** Reads the ambient gas level (analog). Activates a local alarm (buzzer) and sends a notification to the Python server when a predefined threshold is exceeded.
* **Blynk Integration:**
    * Connects to the Blynk server.
    * Sends the gas sensor value to the Blynk application (virtual pin V0).
    * Receives and updates the gas alarm threshold value from the Blynk application (virtual pin V1).
    * Sends an event log to Blynk mobilité in case of a gas alarm.
* **Telegram Notifications:** Sends a message to a specified user via Telegram in case of a gas alarm.
* **Status LED (NeoPixel):**
    * Indicates the current system status (Wi-Fi connection, audio playback, microphone activity, gas alarm) with different colors.
* **Error Handling and Debug:** Provides detailed debug messages via the serial port.

## Python Server Code

The `PythonCodeRidvanSevik-211401051.py` file contains the Python server application chạy on a computer. Its main functions are:

* **UDP Server:**
    * `UDP_PORT`: Listens for audio data from the ESP32.
    * `UDP_PORT_GAS`: Listens for gas sensor data from the ESP32.
* **Audio Processing:**
    * **Google Cloud Speech-to-Text (STT):** Converts raw audio data from the ESP32 into text.
        * Listens for a hotword (e.g., "hello").
        * Listens for user commands after the hotword is detected.
    * **gTTS (Google Text-to-Speech):** Converts text responses into audio files.
    * **Pydub:** Converts audio files to PCM format playable by the ESP32 and adjusts volume.
* **Artificial Intelligence Integration (Groq API):**
    * Sends text commands received from the user to a language model via the Groq API (e.g., Llama 3 model).
    * Receives responses from the language model.
    * Manages conversation history to produce more contextual responses.
    * Provides current gas sensor data from the ESP32 as context to the LLM, so the LLM can use this information if the user asks about the gas status.
* **Gas Data Processing and Alerting:**
    * Receives and stores gas data periodically sent from the ESP32.
    * If the gas level exceeds a certain threshold (e.g., 800) and/or is rising, it generates a specific voice message (e.g., "Gas level is above 800 and rising.") and sends it to the ESP32 to alert the user audibly. This is an additional audio alert to the ESP32's own buzzer alarm.
* **Interaction Logic:**
    * **Hotword Detection:** Continuously waits for a hotword.
    * **Command Detection:** Receives user commands after hotword detection.
    * **Follow-up Mode:** Can listen for new commands for a short period after a successful interaction without requiring the hotword.
    * **Finish Word:** Allows the user to end the interaction using a word like "finish".
    * **Silence and Timeouts:** Ends listening if there's silence for a certain period during command listening or if the maximum listening time is exceeded.
* **Multithreading:**
    * Uses a separate thread to listen for gas data.
    * Can create temporary threads to send gas alert audio asynchronously.
* **Configuration and API Keys:** Retrieves sensitive information like Google API key and Groq API key from environment variables.
* **Logging:** Logs system events and errors in detail.

## System Interaction

1.  **Initialization:**
    * The ESP32 connects to Wi-Fi and Blynk, and starts listening on UDP ports.
    * The Python server starts listening on UDP ports and initializes API clients.

2.  **Voice Interaction:**
    * The user speaks into the microphone connected to the ESP32.
    * The ESP32 captures the sound and sends raw audio data to the Python server via UDP.
    * The Python server continuously receives this audio stream and tries to detect a "hotword" (e.g., "hello") using Google STT.
    * When the hotword is detected, the Python server may send an "I'm listening" message (audio) to the ESP32 and then processes subsequent audio as a user command.
    * After the command is converted to text (STT), it is sent to the language model on the Groq API. Gas data can also be provided as additional context to the LLM at this time.
    * The text response from the language model is received.
    * This response is converted to speech using gTTS and then to PCM format (Pydub).
    * The generated PCM audio data is sent to the ESP32 via UDP.
    * The ESP32 plays the incoming audio data through the speaker.

3.  **Gas Detection and Alerting:**
    * The ESP32 periodically reads the MQ2 gas sensor.
    * The reading is sent to Blynk and transmitted to the Python server via UDP.
    * If the gas level exceeds the threshold defined on the ESP32:
        * The ESP32 activates the local buzzer.
        * The ESP32 sends notifications via Telegram and Blynk.
    * The Python server also receives gas data from the ESP32. If an additional alert condition defined on the Python side is met (e.g., above 800 ppm and rising trend):
        * The Python server generates a custom voice alert message (e.g., "Gas level is high and increasing!").
        * This voice alert is sent to the ESP32 to be played through the speaker.

This system provides a solution that combines hardware (ESP32, sensors) and software (Python, cloud APIs) for both voice-controlled interaction and environmental safety monitoring.