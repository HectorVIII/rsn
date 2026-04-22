import time
import queue
import threading
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import soundfile as sf
import speech_recognition as sr
import numpy as np


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # ===== Parameters =====
        self.declare_parameter('publish_topic', '/voice_target_instrument')

        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('block_duration', 0.1)
        self.declare_parameter('silence_threshold', 0.01)
        self.declare_parameter('silence_seconds_end', 1.0)
        self.declare_parameter('max_record_seconds', 5.0)
        self.declare_parameter('device', None)

        self.declare_parameter('enable_debug_log', True)
        self.declare_parameter('publish_raw_text', True)
        self.declare_parameter('raw_text_topic', '/voice_recognized_text')

        # ===== Read parameters =====
        self.publish_topic = str(self.get_parameter('publish_topic').value)

        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.block_duration = float(self.get_parameter('block_duration').value)
        self.silence_threshold = float(self.get_parameter('silence_threshold').value)
        self.silence_seconds_end = float(self.get_parameter('silence_seconds_end').value)
        self.max_record_seconds = float(self.get_parameter('max_record_seconds').value)
        self.device = self.get_parameter('device').value

        self.enable_debug_log = bool(self.get_parameter('enable_debug_log').value)
        self.publish_raw_text = bool(self.get_parameter('publish_raw_text').value)
        self.raw_text_topic = str(self.get_parameter('raw_text_topic').value)

        # ===== Publishers =====
        self.target_pub = self.create_publisher(String, self.publish_topic, 10)
        self.raw_text_pub = self.create_publisher(String, self.raw_text_topic, 10)

        # ===== Voice state =====
        self.audio_queue = queue.Queue()
        self.running = True

        # ===== Mapping: parsed instrument -> YOLO class name =====
        self.voice_to_vision_class = {
            "scalpel_handle": "SCALPEL_HANDLE",
            "needle_holder": "NEEDLE_HOLDER",
            "tissue_forceps": "TISSUE_FORCEPS",
            "retractor": "RETRACTOR",
            "metzenbaum_scissors": "METZENBAUM_SCISSORS",
        }

        # ===== Background thread =====
        self.worker_thread = threading.Thread(
            target=self.voice_command_worker,
            daemon=True
        )
        self.worker_thread.start()

        self.get_logger().info('voice_command_node started.')
        self.get_logger().info(f'Publish topic: {self.publish_topic}')
        self.get_logger().info(f'Raw text topic: {self.raw_text_topic}')
        self.get_logger().info(f'Sample rate: {self.sample_rate}')
        self.get_logger().info(f'Block duration: {self.block_duration}')
        self.get_logger().info(f'Silence threshold: {self.silence_threshold}')
        self.get_logger().info(f'Silence end seconds: {self.silence_seconds_end}')
        self.get_logger().info(f'Max record seconds: {self.max_record_seconds}')
        self.get_logger().info('Example command: hi sparc give me the forceps')

    # ============================================================
    # Helpers
    # ============================================================
    def log_debug(self, text: str) -> None:
        if self.enable_debug_log:
            self.get_logger().info(text)

    def normalize_voice_text(self, text: str) -> str:
        text = text.lower().strip()

        replacements = {
            "spark": "sparc",
            "scalpel handel": "scalpel handle",
            "scalpelhandle": "scalpel handle",
            "needleholder": "needle holder",
            "tissueforceps": "tissue forceps",
            "metzenbaumscissors": "metzenbaum scissors",
        }

        for old, new in replacements.items():
            text = text.replace(old, new)

        return text

    def parse_voice_command(self, text: str):
        text = self.normalize_voice_text(text)

        command_aliases = [
            "give me",
            "bring me",
            "hand me",
        ]

        instrument_aliases = {
            "scalpel_handle": ["scalpel handle", "scalpel"],
            "needle_holder": ["needle holder", "needleholder", "holder"],
            "tissue_forceps": ["tissue forceps", "forceps"],
            "retractor": ["retractor"],
            "metzenbaum_scissors": ["metzenbaum scissors", "metzenbaum", "scissors"],
        }

        has_command = any(cmd in text for cmd in command_aliases)

        matched_instrument = None
        for canonical_name, aliases in instrument_aliases.items():
            for alias in aliases:
                if alias in text:
                    matched_instrument = canonical_name
                    break
            if matched_instrument:
                break

        if has_command and matched_instrument:
            return {
                "ok": True,
                "command": "give_me",
                "instrument": matched_instrument,
                "normalized_text": text,
            }

        return {
            "ok": False,
            "command": None,
            "instrument": None,
            "normalized_text": text,
        }

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'audio status: {status}')
        self.audio_queue.put(indata.copy())

    def rms(self, chunk: np.ndarray) -> float:
        return float(np.sqrt(np.mean(np.square(chunk))))

    def clear_audio_queue(self) -> None:
        while not self.audio_queue.empty():
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                break

    def record_one_utterance(self):
        self.clear_audio_queue()

        recorded_chunks = []
        started = False
        silence_time = 0.0
        max_blocks = int(self.max_record_seconds / self.block_duration)

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            dtype='float32',
            blocksize=int(self.sample_rate * self.block_duration),
            device=self.device,
            callback=self.audio_callback,
        ):
            for _ in range(max_blocks):
                if not self.running:
                    return None

                chunk = self.audio_queue.get()
                level = self.rms(chunk)

                if not started:
                    if level > self.silence_threshold:
                        started = True
                        recorded_chunks.append(chunk)
                        silence_time = 0.0
                else:
                    recorded_chunks.append(chunk)

                    if level > self.silence_threshold:
                        silence_time = 0.0
                    else:
                        silence_time += self.block_duration
                        if silence_time >= self.silence_seconds_end:
                            break

        if not recorded_chunks:
            return None

        return np.concatenate(recorded_chunks, axis=0)

    def recognize_audio(self, audio_array: np.ndarray) -> str:
        recognizer = sr.Recognizer()

        with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as tmp:
            sf.write(tmp.name, audio_array, self.sample_rate)
            with sr.AudioFile(tmp.name) as source:
                audio_data = recognizer.record(source)

            return recognizer.recognize_google(audio_data)

    def publish_raw_text_msg(self, text: str) -> None:
        if not self.publish_raw_text:
            return

        msg = String()
        msg.data = text
        self.raw_text_pub.publish(msg)

    def publish_target(self, target_cls: str) -> None:
        msg = String()
        msg.data = target_cls
        self.target_pub.publish(msg)
        self.get_logger().info(f'Published voice target: {target_cls}')

    # ============================================================
    # Worker
    # ============================================================
    def voice_command_worker(self):
        self.get_logger().info('Voice worker thread started.')

        while self.running:
            try:
                audio = self.record_one_utterance()
                if audio is None:
                    continue

                text = self.recognize_audio(audio)
                self.publish_raw_text_msg(text)

                result = self.parse_voice_command(text)

                self.log_debug(f'[VOICE] recognized: {text}')
                self.log_debug(f'[VOICE] normalized: {result["normalized_text"]}')

                if result['ok']:
                    target_cls = self.voice_to_vision_class.get(result['instrument'])
                    if target_cls is None:
                        self.get_logger().warn(
                            f'No vision mapping for instrument: {result["instrument"]}'
                        )
                        continue

                    self.publish_target(target_cls)
                    self.log_debug(
                        f'[VOICE] command={result["command"]} target={target_cls}'
                    )
                else:
                    self.log_debug('[VOICE] No valid command parsed.')

            except sr.UnknownValueError:
                self.get_logger().warn('Could not understand audio.')
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition service error: {e}')
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Voice worker error: {e}')
                time.sleep(0.5)

    def destroy_node(self):
        self.get_logger().info('Shutting down voice_command_node...')
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = VoiceCommandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()