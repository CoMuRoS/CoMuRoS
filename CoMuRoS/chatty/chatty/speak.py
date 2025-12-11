#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import edge_tts
from pydub import AudioSegment
import subprocess
import tempfile
import os
import re
import asyncio
import threading
import random
from queue import Queue, Empty

class TTSSpeaker(Node):
    def __init__(self):
        super().__init__('tts_speaker')

        # Parameters
        self.declare_parameter('voice_name', 'en-CA-ClaraNeural')
        self.declare_parameter('speech_rate', '-10%')

        self.sub = self.create_subscription(String, '/chat/output', self.chat_output_callback, 10)
        # self.face_pub = self.create_publisher(Int32, '/face_mode', 10)

        # --- Precompiled regex for speed
        self.task_regex = re.compile(r'Task Manager:\s*(.+)', re.IGNORECASE | re.DOTALL)


        # --- Filter keywords and fallback attention sentences
        self.skip_keywords = ["independent task", "plan", "insdependent"]
        self.never_speak_words = ["all the helps are used", "multi-robot task completed successfully","excellent work"]
        
        self.attention_list = [
            # "Let me give you a hint!",
            # "Here's a small clue check this out!",
            # "Here's a little hint for you!",
            # "A small hint coming your way!",
            "Hey, You may try this!",

        ]


        self.tts_queue = Queue()
        self.stop_event = threading.Event()
        self.loop = asyncio.new_event_loop()
        self.worker_thread = threading.Thread(target=self.tts_worker, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("TTSSpeaker active: Listening for Task Manager messages on /chat/output")

    def chat_output_callback(self, msg: String):
        """Handle new chat message and queue TTS if relevant."""
        def sanitize_for_tts(text: str) -> str:
            """Remove characters that might hinder text-to-speech processing."""
            import re
            # Replace all newline variants, tabs, and disruptive characters with a single space
            text = re.sub(r'[\n\r\f\v\t*(){}[\]<>]+', ' ', text)
            # Normalize all whitespace (including multiple spaces) to a single space
            text = re.sub(r'\s+', ' ', text)
            # Strip leading/trailing spaces
            return text.strip()

        match = self.task_regex.search(msg.data)
        if match:
            text = match.group(1).strip()
            lower_text = text.lower()

            sanitized_text = sanitize_for_tts(lower_text)
            self.get_logger().info(f"Sanitized text: {repr(sanitized_text)}")  # Debug log with repr
                     
            if any(nsw in sanitized_text for nsw in self.never_speak_words):
                return

            # If text contains skip keywords → speak attention grabber instead
            if any(kw in sanitized_text for kw in self.skip_keywords):
                self.get_logger().info(f"Skipping due to keyword match: {sanitized_text}")
                attention_text = random.choice(self.attention_list)
                self.tts_queue.put(attention_text)
                self.get_logger().info(f"Triggered attention phrase: {attention_text}")
                return
            
            self.tts_queue.put(sanitized_text)
            self.get_logger().info(f"Queued for TTS: {sanitized_text[:150]}{'...' if len(sanitized_text) > 150 else ''}")

    def tts_worker(self):
        """Background worker for sequential TTS processing."""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._tts_loop())

    async def _tts_loop(self):
        """Async loop to continuously process queued text."""
        while not self.stop_event.is_set():
            try:
                text = await asyncio.to_thread(self.tts_queue.get, True, 0.2)
                await self.speak_text(text)
                self.tts_queue.task_done()
            except Empty:
                await asyncio.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f"TTS loop error: {e}")

    async def speak_text(self, text: str):
        """Convert text to speech and play sequentially."""
        voice = self.get_parameter('voice_name').value
        speech_rate = self.get_parameter('speech_rate').value

        self.publish_face_mode(-1)

        mp3_path, wav_path = None, None
        try:
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as mp3_file:
                mp3_path = mp3_file.name
            wav_path = mp3_path.replace('.mp3', '.wav')

            # Generate TTS
            # await edge_tts.Communicate(text, voice).save(mp3_path)
            communicate = edge_tts.Communicate(text, voice, rate=speech_rate)
            await communicate.save(mp3_path)

            # Convert and play
            AudioSegment.from_mp3(mp3_path).export(wav_path, format='wav')
            subprocess.run(["aplay", "-D", "default", "--quiet", wav_path], check=True)

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
        finally:
            # self.publish_face_mode(0)
            for f in [mp3_path, wav_path]:
                if f and os.path.exists(f):
                    os.remove(f)
            self.get_logger().info("Speech complete.")

    # def publish_face_mode(self, value: int):
    #     msg = Int32()
    #     msg.data = value
    #     self.face_pub.publish(msg)

    def destroy_node(self):
        """Shutdown cleanly."""
        self.stop_event.set()
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.worker_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSSpeaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()