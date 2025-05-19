import socket
import os
import threading
import sys
import io
import time
import json
import logging

# ... (Your existing imports will remain here) ...
# Required libraries for TTS and FFmpeg check
try:
    from gtts import gTTS
    from pydub import AudioSegment
    # pydub needs FFmpeg or Libav.
    try:
        AudioSegment.converter = AudioSegment.ffmpeg or AudioSegment.avconv
        if not AudioSegment.converter:
            pass
    except Exception as e_pydub_converter:
        print(f"There might be an issue finding Pydub converter (ffmpeg/avconv): {e_pydub_converter}")
except ImportError:
    print("ERROR: Please install 'gtts' and 'pydub' libraries. (pip install gtts pydub)")
    print("Additionally, for pydub to work, FFmpeg or Libav must be installed on your system and in the PATH.")
    exit(1)

# Google Cloud libraries
try:
    from google.cloud import speech
    from google.api_core.client_options import ClientOptions
except ImportError:
    print("ERROR: Please install the 'google-cloud-speech' library. (pip install google-cloud-speech)")
    exit(1)

# requests library (for Groq API)
try:
    import requests
except ImportError:
    print("ERROR: Please install the 'requests' library. (pip install requests)")
    exit(1)

# --- Logging Settings ---
LOG_LEVEL = logging.INFO # logging.DEBUG for more detailed logs
logging.basicConfig(
    level=LOG_LEVEL,
    format='%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

# --- GLOBAL SETTINGS ---
# API Keys (From Environment Variables)
GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY")
GROQ_API_KEY = os.environ.get("GROQ_API_KEY")

# UDP Settings
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
UDP_PORT_GAS = 12346
SOCKET_BUFFER_SIZE = 4096
ESP32_AUDIO_CHUNK_SIZE = 1400
ESP32_AUDIO_PACKET_DELAY = 0.040

# Audio Settings
SAMPLE_RATE = 16000
VOLUME_REDUCTION_DB = 10.0
MAX_LISTEN_SECONDS = 50.0
COMMAND_SILENCE_TIMEOUT_SECONDS = 25.0
is_in_follow_up_mode = False
last_successful_interaction_time = 0.0
FOLLOW_UP_MODE_TIMEOUT_SECONDS = 15.0
COMMAND_FOLLOWUP_SILENCE_TIMEOUT_SECONDS = 10.0
# Hotword and Language Settings
HOTWORD = "hello"
HOTWORD_LANGUAGE_CODE = "en-US"
FINISH_WORD = "finish"
STT_LANGUAGE_CODE = "en-US"
TTS_LANGUAGE_CODE = "en"

# Groq API Settings
GROQ_MODEL = "llama3-70b-8192"
GROQ_TEMPERATURE = 0.7
GROQ_MAX_TOKENS = 1024
GROQ_TIMEOUT_SECONDS = 30

# Conversation History
MAX_CONVERSATION_HISTORY_PAIRS = 5
conversation_history = [
    {
        "role": "system",
        "content": f"You are a helpful AI assistant. Your responses should be in {STT_LANGUAGE_CODE.split('-')[0]}."
                   " Keep your answers concise and short"
    }
]

# --- NEW: Gas Alert Settings ---
GAS_HIGH_THRESHOLD = 800
GAS_ALERT_MESSAGE_ABOVE_800_RISING_EN = "Gas level is above 800 and rising."
GAS_ALERT_MESSAGE_ABOVE_800_EN = "Gas level is above 800."

# --- Global Variables ---
speech_client = None
udp_socket = None
udp_socket_gas = None
last_known_client_address = None
stop_audio_event = threading.Event()
main_loop_stop_event = threading.Event()
gas_listener_stop_event = threading.Event()

latest_gas_data = {
    "value": None,
    "alert": None, # This is the boolean alert directly from ESP32
    "timestamp": 0
}
gas_data_lock = threading.Lock()
GAS_DATA_STALE_SECONDS = 30

# --- NEW: Global Variables for "Above 800" Gas Alert Logic ---
gas_alert_for_800_active = False # Tracks if the specific "above 800" audio alert is currently active
previous_gas_value_for_800_alert_logic = None # Stores the last gas value for this alert's "rising" detection

def initialize_clients_and_socket():
    global speech_client, udp_socket, udp_socket_gas

    if not GOOGLE_API_KEY:
        logger.critical("ERROR: GOOGLE_API_KEY environment variable is not set!")
        return False
    if not GROQ_API_KEY:
        logger.critical("ERROR: GROQ_API_KEY environment variable is not set!")
        return False

    try:
        logger.info("Initializing Google Speech client...")
        client_options = ClientOptions(api_key=GOOGLE_API_KEY)
        speech_client = speech.SpeechClient(client_options=client_options)
        logger.info("Google Speech client initialized successfully.")
    except Exception as e:
        logger.critical(f"Failed to initialize Google Speech client. Error: {e}", exc_info=True)
        return False

    try:
        logger.info(f"Binding main UDP socket to {UDP_IP}:{UDP_PORT} (for audio)...")
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind((UDP_IP, UDP_PORT))
        logger.info(f"Main UDP socket successfully bound to {UDP_IP}:{UDP_PORT}.")
    except socket.error as e:
        logger.critical(f"Error binding main UDP socket: {e}", exc_info=True)
        return False

    try:
        logger.info(f"Binding gas data UDP socket to {UDP_IP}:{UDP_PORT_GAS}...")
        udp_socket_gas = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket_gas.bind((UDP_IP, UDP_PORT_GAS))
        logger.info(f"Gas data UDP socket successfully bound to {UDP_IP}:{UDP_PORT_GAS}.")
    except socket.error as e:
        logger.critical(f"Error binding gas data UDP socket: {e}", exc_info=True)
        udp_socket_gas = None # Ensure it's None if binding fails
    return True

# --- NEW: Asynchronous Task for Sending Gas Alert Audio ---
def _send_gas_alert_audio_task(message, lang, client_addr_at_moment_of_alert, sample_rate, volume_reduction):
    """Helper function to run TTS and send audio in a separate thread for gas alerts."""
    logger.info(f"Async task: Generating audio for gas alert: '{message}' to {client_addr_at_moment_of_alert}")

    if not client_addr_at_moment_of_alert:
        logger.warning("Async task: ESP32 address was unknown when gas alert was triggered. Cannot send audio.")
        return

    pcm_audio = text_to_pcm_audio(
        message,
        lang=lang,
        sample_rate=sample_rate,
        volume_reduction_db=volume_reduction
    )

    if pcm_audio:
        # Use the modified send_audio_to_esp32 with the captured address
        send_audio_to_esp32(pcm_audio, target_address_override=client_addr_at_moment_of_alert)
    else:
        logger.error(f"Async task: Failed to generate PCM for gas alert message: '{message}'")

def listen_for_gas_data():
    global latest_gas_data, gas_data_lock, udp_socket_gas
    # <<< MODIFIED: Access new global variables for this function's specific alert logic >>>
    global gas_alert_for_800_active, previous_gas_value_for_800_alert_logic
    global last_known_client_address # To capture the address for the async task

    if not udp_socket_gas:
        logger.warning("Gas data socket could not be initialized, so gas data cannot be listened for.")
        return

    logger.info(f"Waiting for gas data from ESP32 on UDP port {UDP_PORT_GAS}...")
    while not gas_listener_stop_event.is_set():
        try:
            udp_socket_gas.settimeout(1.0) # Non-blocking wait
            data, addr = udp_socket_gas.recvfrom(SOCKET_BUFFER_SIZE)
            if data:
                data_str = data.decode('utf-8').strip()
                logger.debug(f"Gas data received ({addr}): {data_str}")
                parts = data_str.split(',')
                gas_val_str = None
                alert_str_from_esp = None # Alert boolean string from ESP32

                if len(parts) == 2:
                    if parts[0].startswith("gas:"):
                        gas_val_str = parts[0][len("gas:"):]
                    if parts[1].startswith("alert:"):
                        alert_str_from_esp = parts[1][len("alert:"):]

                if gas_val_str is not None and alert_str_from_esp is not None:
                    try:
                        current_gas_reading = int(gas_val_str)
                        is_alert_from_esp_bool = alert_str_from_esp.lower() == "true"

                        with gas_data_lock: # Update the shared latest_gas_data
                            latest_gas_data["value"] = current_gas_reading
                            latest_gas_data["alert"] = is_alert_from_esp_bool # ESP32's own alert status
                            latest_gas_data["timestamp"] = time.time()
                        logger.debug(f"Gas data updated: Value={current_gas_reading}, ESP_Alert_Flag={is_alert_from_esp_bool}")

                        # --- MODIFIED: Logic for 'above 800' audio alert ---
                        is_currently_rising_for_alert = False
                        if previous_gas_value_for_800_alert_logic is not None and \
                           current_gas_reading > previous_gas_value_for_800_alert_logic:
                            is_currently_rising_for_alert = True

                        if current_gas_reading > GAS_HIGH_THRESHOLD:
                            if not gas_alert_for_800_active: # If alert is not already active for this high period
                                audio_alert_message_to_send = ""
                                if is_currently_rising_for_alert:
                                    audio_alert_message_to_send = GAS_ALERT_MESSAGE_ABOVE_800_RISING_EN
                                    logger.info(f"Gas level ({current_gas_reading}) is above {GAS_HIGH_THRESHOLD} and rising. Triggering audio alert.")
                                else:
                                    # Above threshold, but not rising from immediate previous, or first detection in high state
                                    audio_alert_message_to_send = GAS_ALERT_MESSAGE_ABOVE_800_EN
                                    logger.info(f"Gas level ({current_gas_reading}) is above {GAS_HIGH_THRESHOLD}. Triggering audio alert.")
                                
                                current_esp32_address_for_alert = last_known_client_address # Capture current address for the new thread
                                if current_esp32_address_for_alert:
                                    alert_thread = threading.Thread(
                                        target=_send_gas_alert_audio_task,
                                        args=(
                                            audio_alert_message_to_send,
                                            "en", # Force English for this alert
                                            current_esp32_address_for_alert,
                                            SAMPLE_RATE,
                                            VOLUME_REDUCTION_DB
                                        )
                                    )
                                    alert_thread.daemon = True # Allow program to exit even if this thread is running
                                    alert_thread.start()
                                else:
                                    logger.warning(f"High gas level ({current_gas_reading}) detected for audio alert, but ESP32 address is currently unknown. Audio alert not sent.")
                                gas_alert_for_800_active = True # Mark that audio alert has been issued for this high period
                        
                        elif current_gas_reading <= GAS_HIGH_THRESHOLD: # Gas level is normal or low regarding the 800 threshold
                            if gas_alert_for_800_active: # If it was previously high and alerted (for the 800 threshold)
                                logger.info(f"Gas level ({current_gas_reading}) is now at or below {GAS_HIGH_THRESHOLD}. Resetting 'above 800' audio alert active flag.")
                                gas_alert_for_800_active = False # Reset the flag, ready for the next rise above 800

                        # Update the previous value for the next cycle of this specific "above 800" alert logic
                        previous_gas_value_for_800_alert_logic = current_gas_reading
                        # --- End of modified 'above 800' alert logic ---

                    except ValueError:
                        logger.warning(f"Received gas data is not a valid integer: {gas_val_str}")
                else:
                    logger.warning(f"Received gas data format invalid (expected 'gas:VALUE,alert:STATE'): {data_str}")
        except socket.timeout:
            continue # Normal timeout, loop again to check stop_event
        except Exception as e:
            if not gas_listener_stop_event.is_set(): # Log error only if not intentionally stopping
                logger.error(f"Error while listening for gas data: {e}", exc_info=True)
            time.sleep(0.5) # Avoid rapid looping on persistent error
    logger.info("Gas data listening thread terminated.")


def generate_audio_requests(current_stop_event):
    global last_known_client_address, udp_socket
    while not current_stop_event.is_set():
        try:
            udp_socket.settimeout(0.1)
            data, addr = udp_socket.recvfrom(SOCKET_BUFFER_SIZE)
            if not current_stop_event.is_set() and data:
                if last_known_client_address != addr:
                    logger.info(f"New ESP32 address detected (for audio): {addr}")
                    last_known_client_address = addr
                yield speech.StreamingRecognizeRequest(audio_content=data)
        except socket.timeout:
            if current_stop_event.is_set():
                logger.debug("generate_audio_requests stopping due to event after socket timeout.")
                break
            continue
        except Exception as e:
            if not current_stop_event.is_set():
                logger.error(f"Socket error in generate_audio_requests: {e}", exc_info=True)
            break
    logger.debug(f"generate_audio_requests terminated (event set: {current_stop_event.is_set()}).")


def get_groq_response(user_text):
    global conversation_history, latest_gas_data, gas_data_lock
    logger.info(f"Sending to Groq API (original): \"{user_text}\"")

    user_text_for_api = user_text
    with gas_data_lock:
        gas_value = latest_gas_data["value"]
        is_alert_from_esp = latest_gas_data["alert"] # This is the boolean alert from ESP32
        data_timestamp = latest_gas_data["timestamp"]

    if gas_value is not None and (time.time() - data_timestamp) < GAS_DATA_STALE_SECONDS:
        # Use the ESP32's own alert status for informing the LLM
        status_text = "ALERT (from ESP32)" if is_alert_from_esp else "Normal (from ESP32)"
        gas_info = f"Current gas sensor reading: {gas_value} (ESP32 Status: {status_text})."
        user_text_for_api = f"{gas_info} if the user asks something about this, answer it, but if the user doesn't ask something about this you don't have to say it, the user's query is: {user_text}"
        logger.info(f"Using gas data (age: {time.time() - data_timestamp:.1f}s). Enriched text to be sent to API: \"{user_text_for_api}\"")
    elif gas_value is not None:
        logger.info(f"Gas data is stale (age: {time.time() - data_timestamp:.1f}s), will not be added to LLM prompt.")
    else:
        logger.info("No available gas data, will not be added to LLM prompt.")

    current_conversation_for_api = [conversation_history[0]]
    user_assistant_pairs = []
    for i in range(1, len(conversation_history), 2):
        if i + 1 < len(conversation_history):
            user_assistant_pairs.append(
                (conversation_history[i], conversation_history[i+1])
            )
    
    for user_msg, assistant_msg in user_assistant_pairs[-MAX_CONVERSATION_HISTORY_PAIRS:]:
        current_conversation_for_api.append(user_msg)
        current_conversation_for_api.append(assistant_msg)
    
    current_conversation_for_api.append({"role": "user", "content": user_text_for_api})
    
    logger.debug(f"Messages to be sent to Groq ({len(current_conversation_for_api)} count): {json.dumps(current_conversation_for_api, indent=2)}")

    payload = {
        "model": GROQ_MODEL,
        "messages": current_conversation_for_api,
        "temperature": GROQ_TEMPERATURE,
        "max_tokens": GROQ_MAX_TOKENS,
        "stream": False
    }
    headers = {
        "Authorization": f"Bearer {GROQ_API_KEY}",
        "Content-Type": "application/json"
    }
    api_url = "https://api.groq.com/openai/v1/chat/completions"

    try:
        response = requests.post(api_url, headers=headers, json=payload, timeout=GROQ_TIMEOUT_SECONDS)
        response.raise_for_status()

        response_data = response.json()
        assistant_response_text = response_data["choices"][0]["message"]["content"]
        logger.info(f"Response received from Groq API: \"{assistant_response_text}\"")

        conversation_history.append({"role": "user", "content": user_text}) 
        conversation_history.append({"role": "assistant", "content": assistant_response_text})

        while (len(conversation_history) -1) / 2 > MAX_CONVERSATION_HISTORY_PAIRS:
            conversation_history.pop(1) 
            conversation_history.pop(1) 
        
        logger.debug(f"Updated conversation history ({len(conversation_history)} count): {json.dumps(conversation_history, indent=2)}")
        return assistant_response_text

    except requests.exceptions.Timeout:
        logger.error(f"Groq API request timed out ({GROQ_TIMEOUT_SECONDS} seconds).")
        return "Sorry, I couldn't reach my brain in time."
    except requests.exceptions.HTTPError as e:
        logger.error(f"Groq API HTTP Error: {e.response.status_code} - {e.response.text}", exc_info=True)
        return "Sorry, there was an issue with my connection to the thinking cap."
    except requests.exceptions.RequestException as e:
        logger.error(f"Groq API Connection Error: {e}", exc_info=True)
        return "Sorry, I'm having trouble connecting right now."
    except Exception as e:
        logger.error(f"Unexpected error while processing response from Groq API: {e}", exc_info=True)
        return "Sorry, a little hiccup happened while I was thinking."


def text_to_pcm_audio(text_to_speak, lang=TTS_LANGUAGE_CODE, sample_rate=SAMPLE_RATE, volume_reduction_db=VOLUME_REDUCTION_DB):
    tts_conversion_start_time = time.time()
    logger.info(f"Text for TTS: \"{text_to_speak}\" (Language: {lang}, Volume Reduction: {volume_reduction_db}dB)")
    try:
        tts = gTTS(text=text_to_speak, lang=lang, slow=False)
        mp3_fp = io.BytesIO()
        tts.write_to_fp(mp3_fp)
        mp3_fp.seek(0)

        audio_segment = AudioSegment.from_file(mp3_fp, format="mp3")

        if volume_reduction_db > 0:
            logger.debug(f"Reducing volume by {volume_reduction_db} dB.")
            audio_segment = audio_segment - volume_reduction_db
        elif volume_reduction_db < 0:
            logger.debug(f"Increasing volume by {-volume_reduction_db} dB.")
            audio_segment = audio_segment + (-volume_reduction_db)

        audio_segment = audio_segment.set_channels(1)
        audio_segment = audio_segment.set_frame_rate(sample_rate)
        audio_segment = audio_segment.set_sample_width(2) # 16-bit

        pcm_data = audio_segment.raw_data
        duration_seconds = len(pcm_data) / (sample_rate * 1 * 2.0) # 1 channel, 2 bytes per sample
        processing_time = time.time() - tts_conversion_start_time
        logger.info(
            f"PCM audio data created: {len(pcm_data)} bytes, "
            f"Audio Duration: {duration_seconds:.2f}s, {sample_rate}Hz, 16-bit Mono. "
            f"TTS+PCM PROCESSING TIME: {processing_time:.2f} SECONDS"
        )
        return pcm_data
    except Exception as e:
        if "Language not supported" in str(e) or "failed to read" in str(e).lower():
            logger.error(f"TTS Error: Language '{lang}' may not be supported by gTTS or there is an issue with FFmpeg/Libav. Details: {e}", exc_info=True)
            if "FFMPEG_PATH" not in os.environ and AudioSegment.converter is None:
                logger.error("It seems FFmpeg/Libav is not installed and in PATH, or pydub could not find it.")
        else:
            logger.error(f"TTS or audio conversion error: {e}", exc_info=True)
        return None

# --- MODIFIED: send_audio_to_esp32 to accept target_address_override ---
def send_audio_to_esp32(pcm_audio_to_send, target_address_override=None):
    global last_known_client_address, udp_socket # Added globals for clarity

    # Determine the address to use
    address_to_use = target_address_override if target_address_override else last_known_client_address

    if not pcm_audio_to_send:
        logger.warning("No PCM audio data to send to ESP32.")
        return
    if not address_to_use:
        logger.warning(f"ESP32 address is unknown (globally or passed target_address_override is None), audio cannot be sent.")
        return

    logger.info(f"--> Sending {len(pcm_audio_to_send)} bytes of PCM audio to ESP32 ({address_to_use})...")
    total_chunks = (len(pcm_audio_to_send) + ESP32_AUDIO_CHUNK_SIZE - 1) // ESP32_AUDIO_CHUNK_SIZE
    logger.info(f"       Will be sent in {total_chunks} packets.")

    for i in range(0, len(pcm_audio_to_send), ESP32_AUDIO_CHUNK_SIZE):
        chunk = pcm_audio_to_send[i:i + ESP32_AUDIO_CHUNK_SIZE]
        try:
            udp_socket.sendto(chunk, address_to_use)
            if ESP32_AUDIO_PACKET_DELAY > 0:
                time.sleep(ESP32_AUDIO_PACKET_DELAY)
        except socket.error as e:
            logger.error(f"Socket error while sending audio packet to ESP32 ({address_to_use}): {e}", exc_info=True)
            break # Stop sending if an error occurs
    logger.info(f"       All audio packets hopefully sent to ESP32 ({address_to_use}).")


def process_transcription_and_respond(final_transcript_text):
    if not final_transcript_text:
        logger.info("Empty transcript received, no action will be taken.")
        return

    logger.info(f"Final transcript to process: \"{final_transcript_text}\"")
    groq_assistant_text_response = get_groq_response(final_transcript_text)

    if groq_assistant_text_response:
        logger.info(f"Assistant (Text): {groq_assistant_text_response}")
        pcm_audio_to_send = text_to_pcm_audio(
            groq_assistant_text_response,
            lang=TTS_LANGUAGE_CODE,
            sample_rate=SAMPLE_RATE,
            volume_reduction_db=VOLUME_REDUCTION_DB
        )
        send_audio_to_esp32(pcm_audio_to_send) # Uses global last_known_client_address by default


def listen_for_hotword(current_stop_event):
    global speech_client, last_known_client_address # Added globals for clarity
    logger.info(f"\nWaiting for '{HOTWORD}' command ({HOTWORD_LANGUAGE_CODE})...")

    if not last_known_client_address:
        # This warning is okay, generate_audio_requests will try to discover it.
        logger.warning("ESP32 address is unknown at the start of hotword listening. Will attempt to detect.")
        pass # generate_audio_requests will update it if/when audio comes in

    stt_config_hotword = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=SAMPLE_RATE,
        language_code=HOTWORD_LANGUAGE_CODE,
    )
    streaming_stt_config_hotword = speech.StreamingRecognitionConfig(
        config=stt_config_hotword,
        interim_results=True 
    )

    audio_stream_generator = generate_audio_requests(current_stop_event)
    try:
        responses = speech_client.streaming_recognize(streaming_stt_config_hotword, audio_stream_generator)
        for response in responses:
            if current_stop_event.is_set():
                logger.info("Hotword listening stopped externally.")
                break

            if not response.results: continue
            result = response.results[0]
            if not result.alternatives: continue

            transcript_segment = result.alternatives[0].transcript.strip().lower()
            
            # Basic interim display for hotword
            sys.stdout.write('\r' + ' ' * 60 + '\r') # Clear previous line
            sys.stdout.write(f"Hotword scan: ...{transcript_segment[-30:]}") # Show tail
            sys.stdout.flush()

            if HOTWORD in transcript_segment:
                logger.info(f"'{HOTWORD}' command detected!")
                sys.stdout.write('\r' + ' ' * 80 + '\r') 
                sys.stdout.flush()
                print(f"'{HOTWORD}' command detected!") 
                return True 

    except Exception as e_stream:
        # Clear any partial hotword line from stdout
        sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()
        if "rst stream" in str(e_stream).lower() or "service unavailable" in str(e_stream).lower() or "internal" in str(e_stream).lower():
            logger.warning(f"Google STT connection/service issue during hotword listening: {e_stream}. Will retry hotword loop.")
        elif "inactive" in str(e_stream).lower() or "client_stream_inactive" in str(e_stream).lower():
            logger.debug(f"STT stream became inactive during hotword listening (could be silence or end of data): {e_stream}")
        elif "maximum allowed stream duration" in str(e_stream).lower():
            logger.info("Hotword listening STT exceeded maximum stream duration, will restart hotword loop.")
        else:
            logger.error(f"Unexpected error during hotword listening: {e_stream}", exc_info=LOG_LEVEL==logging.DEBUG)
    finally:
        # Ensure any partial line is cleared if loop terminates
        sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()
    return False


def listen_for_command_and_respond(is_current_follow_up=False):
    global stop_audio_event, speech_client, last_known_client_address, FINISH_WORD
    global COMMAND_SILENCE_TIMEOUT_SECONDS, MAX_LISTEN_SECONDS, COMMAND_FOLLOWUP_SILENCE_TIMEOUT_SECONDS
    global TTS_LANGUAGE_CODE, SAMPLE_RATE, VOLUME_REDUCTION_DB

    logger.info(f"\nListening for command... (Follow-up: {is_current_follow_up})")
    stop_audio_event.clear() # Reset for this listening session
    start_time = time.time()
    last_meaningful_activity_time = start_time # For silence detection

    current_silence_timeout = COMMAND_FOLLOWUP_SILENCE_TIMEOUT_SECONDS if is_current_follow_up else COMMAND_SILENCE_TIMEOUT_SECONDS

    stt_config_command = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=SAMPLE_RATE,
        language_code=STT_LANGUAGE_CODE,
    )
    streaming_stt_config_command = speech.StreamingRecognitionConfig(
        config=stt_config_command,
        interim_results=True
    )

    if is_current_follow_up:
        # No audio prompt for follow-up, or a very subtle one if desired later
        logger.debug("In follow-up mode, no 'I'm listening' prompt sent.")
        pass
    else:
        prompt_text = "I'm listening." if TTS_LANGUAGE_CODE.startswith("en") else "Dinliyorum."
        pcm_prompt_sound = text_to_pcm_audio(prompt_text, lang=TTS_LANGUAGE_CODE.split('-')[0])
        if pcm_prompt_sound:
            send_audio_to_esp32(pcm_prompt_sound) # Uses global last_known_client_address

    audio_stream_generator_command = generate_audio_requests(stop_audio_event)
    final_transcript_text = ""
    transcript_processed_for_llm = False
    return_status = "NO_INPUT" # Default status

    try:
        responses = speech_client.streaming_recognize(streaming_stt_config_command, audio_stream_generator_command)
        for response in responses:
            current_time = time.time()

            if current_time - start_time > MAX_LISTEN_SECONDS:
                if not stop_audio_event.is_set():
                    logger.info(f"Command listening time expired ({MAX_LISTEN_SECONDS} seconds).")
                    stop_audio_event.set()
                # Do not set NO_INPUT here yet, let it break and process any pending final_transcript
                break 

            if stop_audio_event.is_set():
                # If event was set (by timeout, finish word, or external), break to process final transcript
                logger.debug("Stop_audio_event is set, breaking from response loop.")
                break

            if response.results and response.results[0].alternatives:
                last_meaningful_activity_time = current_time # Speech detected
                transcript_segment = response.results[0].alternatives[0].transcript.strip()

                if FINISH_WORD.lower() in transcript_segment.lower():
                    logger.info(f"'{FINISH_WORD}' detected. Ending interaction for this turn.")
                    sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()
                    print(f"'{FINISH_WORD}' command detected.")
                    if not stop_audio_event.is_set(): stop_audio_event.set()
                    # Don't process the transcript containing "finish", just end.
                    return "USER_FINISHED" # Directly return, main_loop handles this state

                if response.results[0].is_final:
                    logger.debug(f"Command STT (Final Segment): \"{transcript_segment}\"")
                    final_transcript_text += transcript_segment + " "
                    sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()
                    print(f"User (Command Completed): {final_transcript_text.strip()}")
                    
                    # Process immediately on final, then stop this listening turn.
                    if final_transcript_text.strip() and not transcript_processed_for_llm:
                        process_transcription_and_respond(final_transcript_text.strip())
                        transcript_processed_for_llm = True
                        return_status = "PROCESSED"
                    if not stop_audio_event.is_set():
                        logger.info("Final command transcript received, stopping listening for this turn.")
                        stop_audio_event.set() 
                    # Break to exit the loop since we got a final and processed it
                    break 
                else: # Interim result
                    current_display_text = final_transcript_text + transcript_segment
                    sys.stdout.write('\r' + ' ' * 80 + '\r')
                    sys.stdout.write(f"Recognizing (Command): {current_display_text}")
                    sys.stdout.flush()
            
            # Silence timeout check (only if no results in this particular response iteration)
            # Moved this check to be more robust after processing potential results
            if current_time - last_meaningful_activity_time > current_silence_timeout:
                logger.info(f"Silence detected for over {current_silence_timeout} seconds during command listening.")
                sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()
                print(f"Silence detected for {current_silence_timeout}s.")
                if not stop_audio_event.is_set(): stop_audio_event.set()
                # Break to process any accumulated transcript before silence
                break 

        # --- End of response loop ---

        # Ensure any remaining interim line is cleared
        sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()

        # Process any accumulated final_transcript_text if not already processed and loop ended for other reasons (e.g. timeout, silence)
        if final_transcript_text.strip() and not transcript_processed_for_llm:
            logger.info("Processing accumulated transcript at end of command listening...")
            print(f"User (Command Partial/Timeout): {final_transcript_text.strip()}") # Show what was caught
            process_transcription_and_respond(final_transcript_text.strip())
            transcript_processed_for_llm = True
            return_status = "PROCESSED"
        elif not final_transcript_text.strip() and return_status == "NO_INPUT": # No speech at all
            logger.info("No speech was captured during command listening.")
            # Return status remains "NO_INPUT"
        
    except Exception as e_stream:
        sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush() # Clear line
        logger.error(f"Error during command listening stream: {e_stream}", exc_info=LOG_LEVEL==logging.DEBUG)
        if final_transcript_text.strip() and not transcript_processed_for_llm:
            logger.error(f"Processing accumulated transcript due to error: \"{final_transcript_text.strip()}\"")
            process_transcription_and_respond(final_transcript_text.strip())
            return_status = "PROCESSED" # Attempted to process something
        else:
            return_status = "ERROR"
    finally:
        if not stop_audio_event.is_set(): # Ensure it's always set before leaving
            stop_audio_event.set()
        # Final cleanup of the line, just in case
        sys.stdout.write('\r' + ' ' * 80 + '\r'); sys.stdout.flush()

    logger.info(f"listen_for_command_and_respond returning with status: {return_status}")
    return return_status


def main_loop():
    global last_known_client_address, main_loop_stop_event, stop_audio_event, gas_listener_stop_event
    global udp_socket, udp_socket_gas 
    global is_in_follow_up_mode, last_successful_interaction_time 
    global TTS_LANGUAGE_CODE

    # Initial ESP32 address detection (can be brief)
    if not last_known_client_address:
        logger.info("Program starting. Attempting to detect ESP32 address briefly (listening for any audio/gas packet)...")
        detection_start_time = time.time()
        temp_stop_event_addr = threading.Event()
        # Use generate_audio_requests to "ping" for an address by listening for a moment
        audio_gen_for_addr_init = generate_audio_requests(temp_stop_event_addr)
        try:
            next(audio_gen_for_addr_init) # Try to get one packet which sets last_known_client_address
        except StopIteration: # No audio packet received quickly
            logger.debug("No initial audio packet received for address detection.")
        except Exception as e_addr_init:
            logger.debug(f"Error during initial address detection listen: {e_addr_init}")
        finally:
            temp_stop_event_addr.set() # Stop the temporary generator

        # Also consider gas data for address (though less likely to be the *audio* destination)
        # The gas listener thread will update latest_gas_data and potentially log its source address.
        # However, last_known_client_address is specifically for audio.

        if not last_known_client_address:
            # If audio didn't yield an address, wait a very short time for gas thread to potentially find one
            # (though listen_for_gas_data doesn't update last_known_client_address for audio)
            # This initial check is mostly for the audio packets.
            if time.time() - detection_start_time < 2 and not last_known_client_address: # Wait up to 2s
                 time.sleep(0.5) # Brief wait

        if last_known_client_address:
            logger.info(f"Initial ESP32 audio address detected: {last_known_client_address}")
        else:
            logger.warning("ESP32 audio address not detected at startup. Hotword listening will still try.")

    try:
        while not main_loop_stop_event.is_set():
            if not last_known_client_address:
                # If address is still unknown, hotword listening might fail or be delayed.
                # generate_audio_requests within listen_for_hotword will keep trying to get it.
                logger.debug("ESP32 audio address still unknown. Hotword listening will attempt to establish it.")
                # Small pause to allow generate_audio_requests in hotword to potentially connect
                # time.sleep(0.5) # Optional brief pause

            should_listen_for_hotword = True
            if is_in_follow_up_mode:
                time_since_last_interaction = time.time() - last_successful_interaction_time
                if time_since_last_interaction < FOLLOW_UP_MODE_TIMEOUT_SECONDS:
                    logger.info(f"Follow-up mode active. Listening for next command. Timeout in {FOLLOW_UP_MODE_TIMEOUT_SECONDS - time_since_last_interaction:.1f}s.")
                    should_listen_for_hotword = False
                else:
                    logger.info("Follow-up mode timed out. Returning to hotword listening.")
                    is_in_follow_up_mode = False
                    # Fall through to hotword listening

            if should_listen_for_hotword:
                is_in_follow_up_mode = False # Ensure it's off if we are listening for hotword
                hotword_detected = listen_for_hotword(main_loop_stop_event)

                if main_loop_stop_event.is_set(): break 
                if not hotword_detected:
                    # logger.debug("Hotword not detected or error in listening. Retrying hotword loop.")
                    time.sleep(0.1) # Small pause before retrying hotword to prevent tight loop on some errors
                    continue 

            # If hotword was detected OR we are in follow-up mode:
            logger.info("Proceeding to listen for command...")
            interaction_status = listen_for_command_and_respond(is_current_follow_up=is_in_follow_up_mode)

            if main_loop_stop_event.is_set(): break

            if interaction_status == "PROCESSED":
                is_in_follow_up_mode = True
                last_successful_interaction_time = time.time()
                logger.info("Command processed successfully. Follow-up mode activated/refreshed.")
            elif interaction_status == "USER_FINISHED":
                is_in_follow_up_mode = False
                logger.info("User indicated 'finish'. Returning to hotword listening.")
                # Optional: "Okay" sound confirmation
                # pcm_confirm = text_to_pcm_audio("Okay.", lang=TTS_LANGUAGE_CODE.split('-')[0])
                # if pcm_confirm: send_audio_to_esp32(pcm_confirm)
            elif interaction_status == "NO_INPUT":
                logger.info("No input detected during command listening.")
                if not is_in_follow_up_mode : # Only say "didn't catch" if it was the first attempt after hotword
                    no_understand_text = "Sorry, I didn't catch that." if TTS_LANGUAGE_CODE.startswith("en") else "Üzgünüm, sizi duyamadım."
                    pcm_error_sound = text_to_pcm_audio(no_understand_text, lang=TTS_LANGUAGE_CODE.split('-')[0])
                    if pcm_error_sound: send_audio_to_esp32(pcm_error_sound)
                is_in_follow_up_mode = False # No input, so exit follow-up
            elif interaction_status == "ERROR":
                is_in_follow_up_mode = False # Error, so exit follow-up
                logger.warning("Error during command processing. Returning to hotword listening.")
                # Optional: "Something went wrong" sound
                # error_text = "Sorry, something went wrong." if TTS_LANGUAGE_CODE.startswith("en") else "Bir sorun oluştu."
                # pcm_gen_error = text_to_pcm_audio(error_text, lang=TTS_LANGUAGE_CODE.split('-')[0])
                # if pcm_gen_error: send_audio_to_esp32(pcm_gen_error)
            else: # Unknown status
                is_in_follow_up_mode = False
                logger.error(f"Unhandled interaction status: '{interaction_status}'. Defaulting to hotword listening.")
            
            if main_loop_stop_event.is_set(): break
            
    except KeyboardInterrupt:
        logger.info("\nProgram terminating via Ctrl+C...")
    except Exception as e:
        logger.critical(f"Unexpected critical error in main loop: {e}", exc_info=True)
    finally:
        logger.info("Main loop ending. Setting stop events for all threads.")
        main_loop_stop_event.set()
        stop_audio_event.set() # Ensure any active audio generation/STT stops
        gas_listener_stop_event.set() # Signal gas listener thread to stop

        if udp_socket:
            logger.info("Closing main UDP socket...")
            udp_socket.close()
            logger.info("Main UDP socket closed.")
        if udp_socket_gas:
            logger.info("Closing gas data UDP socket...")
            udp_socket_gas.close()
            logger.info("Gas data UDP socket closed.")

if __name__ == "__main__":
    logger.info(f"===== Voice Assistant Server Starting (Hotword: '{HOTWORD}', Finish word: '{FINISH_WORD}') =====")
    
    if not AudioSegment.converter:
        logger.warning("FFmpeg/Libav path for Pydub could not be found automatically by pydub.")
        logger.warning("If you experience issues with audio playback/conversion (e.g., TTS 'failed to read' errors),")
        logger.warning("please ensure FFmpeg or Libav is installed and accessible in your system's PATH.")
    else:
        logger.info(f"Pydub converter found: {AudioSegment.converter}")

    if initialize_clients_and_socket():
        gas_thread = None
        if udp_socket_gas: # Only start if socket was bound
            gas_thread = threading.Thread(target=listen_for_gas_data, daemon=True)
            gas_thread.start()
            logger.info("Gas data listening thread started.")
        else:
            logger.warning("Gas data listening thread NOT started as its UDP socket failed to bind.")

        main_loop() # Start the main interaction loop

        if gas_thread and gas_thread.is_alive():
            logger.info("Waiting for gas listening thread to terminate...")
            gas_listener_stop_event.set() # Ensure event is set if not already
            gas_thread.join(timeout=2.0) # Wait for up to 2 seconds
            if gas_thread.is_alive():
                logger.warning("Gas listening thread did not terminate in time.")
    else:
        logger.critical("Failed to initialize clients or sockets. Server cannot start.")
        
    logger.info(f"===== Voice Assistant Server Stopped =====")