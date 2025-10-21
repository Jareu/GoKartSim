"""
Engine audio system with dynamic pitch adjustment based on RPM.

Provides an EngineAudio class that plays engine.wav with pitch modulation
based on RPM input using sounddevice for real-time audio streaming.
"""

import numpy as np
import sounddevice as sd
import soundfile as sf
from threading import Lock
from pathlib import Path
import os
from typing import Optional


class EngineAudio:
    """
    Plays engine noise with real-time pitch adjustment based on RPM.
    
    The audio pitch is scaled based on the RPM ratio, creating a convincing
    engine sound that changes with throttle input. Uses a circular buffer
    for seamless looping.
    
    Attributes:
        enabled (bool): Whether audio is currently playing
        speed (float): Playback speed multiplier (pitch shift factor)
        rpm (float): Current engine RPM (informational)
    """
    
    def __init__(
        self,
        audio_file: str = "sound/engine.wav",
        min_rpm: float = 1200.0,
        max_rpm: float = 16000.0,
        idle_rpm: float = 1000.0,
        redline_rpm: float = 10000.0,
        base_speed: float = 1.0,
        min_pitch: float = 0.3,
        max_pitch: float = 2.5,
    ):
        """
        Initialize the engine audio system.
        
        Args:
            audio_file: Path to the engine.wav audio file
            min_rpm: Minimum RPM for the engine (default: 0)
            max_rpm: Maximum RPM for the engine (default: 12000)
            idle_rpm: RPM at idle (default: 1000)
            redline_rpm: RPM at redline (default: 10000)
            base_speed: Base playback speed (1.0 = normal speed)
            min_pitch: Minimum pitch multiplier (default: 0.3)
            max_pitch: Maximum pitch multiplier (default: 2.5)
        """
        # Audio file path resolution
        self.audio_file = audio_file
        if not os.path.isabs(audio_file):
            # Try relative to this module
            module_dir = Path(__file__).parent.parent
            resolved_path = module_dir / audio_file
            if resolved_path.exists():
                self.audio_file = str(resolved_path)
            else:
                # Try current working directory
                if Path(audio_file).exists():
                    self.audio_file = audio_file
        
        # RPM configuration
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.idle_rpm = idle_rpm
        self.redline_rpm = redline_rpm
        self.base_speed = base_speed
        self.min_pitch = min_pitch
        self.max_pitch = max_pitch
        
        # Audio state
        self.enabled = False
        self.speed = base_speed
        self.rpm = idle_rpm
        self._speed_lock = Lock()
        
        # Audio data
        self.data: Optional[np.ndarray] = None
        self.sample_rate: Optional[int] = None
        self.stream: Optional[sd.OutputStream] = None
        self.position = 0
        
        # Try to load audio
        self._load_audio()
    
    def _load_audio(self) -> bool:
        """
        Load the engine audio file.
        
        Returns:
            True if audio loaded successfully, False otherwise
        """
        try:
            if not os.path.exists(self.audio_file):
                print(f"Warning: Audio file not found: {self.audio_file}")
                return False
            
            self.data, self.sample_rate = sf.read(
                self.audio_file, dtype='float32'
            )
            
            # Ensure audio is in stereo or mono
            if len(self.data.shape) == 1:
                self.data = self.data.reshape(-1, 1)
            
            print(f"Audio loaded: {self.audio_file}")
            print(f"  Sample rate: {self.sample_rate} Hz")
            print(f"  Duration: {len(self.data) / self.sample_rate:.2f}s")
            print(f"  Channels: {self.data.shape[1]}")
            return True
        except Exception as e:
            print(f"Error loading audio file {self.audio_file}: {e}")
            return False
    
    def _callback(self, outdata: np.ndarray, frames: int, 
                  time_info, status) -> None:
        """
        Audio stream callback function.
        
        Called by sounddevice to fill the audio buffer. Implements
        pitch shifting by varying the playback speed.
        
        Args:
            outdata: Output buffer to fill
            frames: Number of frames requested
            time_info: Time information (unused)
            status: Stream status (unused)
        """
        if status:
            print(f"Audio callback status: {status}")
        
        if self.data is None:
            outdata.fill(0)
            return
        
        # Get current speed (thread-safe)
        with self._speed_lock:
            current_speed = self.speed
        
        # Calculate how many samples to read
        # Speed > 1 plays faster (higher pitch)
        # Speed < 1 plays slower (lower pitch)
        samples_to_read = int(current_speed * frames)
        
        # Initialize output buffer
        chunk = np.zeros((frames, self.data.shape[1]), dtype='float32')
        
        # Read from circular buffer
        data_len = len(self.data)
        
        for i in range(frames):
            # Linear interpolation between samples for smoother playback
            sample_pos = self.position + i * current_speed
            idx_low = int(sample_pos) % data_len
            idx_high = (idx_low + 1) % data_len
            frac = sample_pos - int(sample_pos)
            
            # Interpolate
            chunk[i] = (1 - frac) * self.data[idx_low] + frac * self.data[idx_high]
        
        # Update position for next callback
        self.position = (self.position + samples_to_read) % data_len
        
        outdata[:] = chunk
    
    def set_rpm(self, rpm: float) -> None:
        """
        Update engine RPM and adjust pitch accordingly.
        
        Maps RPM to a pitch multiplier in the range [min_pitch, max_pitch].
        Uses a smooth curve to create realistic pitch variation.
        
        Args:
            rpm: Current engine RPM
        """
        self.rpm = max(self.min_rpm, min(self.max_rpm, rpm))
        
        # Normalize RPM to 0..1 range
        rpm_normalized = (self.rpm - self.idle_rpm) / max(
            1e-6, self.redline_rpm - self.idle_rpm
        )
        rpm_normalized = max(0.0, min(1.0, rpm_normalized))
        
        # Map to pitch multiplier with a smooth curve
        # Uses quadratic easing for more realistic feel
        pitch_ratio = self.min_pitch + (
            rpm_normalized ** 1.2 * (self.max_pitch - self.min_pitch)
        )
        
        # Update speed in a thread-safe manner
        new_speed = self.base_speed * pitch_ratio
        with self._speed_lock:
            self.speed = new_speed
    
    def set_speed(self, speed: float) -> None:
        """
        Directly set the playback speed multiplier.
        
        Args:
            speed: Playback speed (1.0 = normal, 0.5 = half speed, 2.0 = double)
        """
        with self._speed_lock:
            self.speed = max(self.min_pitch, min(self.max_pitch, speed))
    
    def start(self) -> bool:
        """
        Start playing the engine audio.
        
        Returns:
            True if audio started successfully, False otherwise
        """
        if self.data is None:
            print("Error: Audio data not loaded")
            return False
        
        if self.enabled:
            print("Audio already playing")
            return True
        
        try:
            self.position = 0
            self.stream = sd.OutputStream(
                callback=self._callback,
                samplerate=self.sample_rate,
                channels=self.data.shape[1],
                blocksize=2048,
            )
            self.stream.start()
            self.enabled = True
            print("Engine audio started")
            return True
        except Exception as e:
            print(f"Error starting audio stream: {e}")
            return False
    
    def stop(self) -> None:
        """Stop playing the engine audio."""
        if self.stream is not None:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception as e:
                print(f"Error stopping audio stream: {e}")
            finally:
                self.stream = None
                self.enabled = False
                print("Engine audio stopped")
    
    def is_playing(self) -> bool:
        """
        Check if audio is currently playing.
        
        Returns:
            True if audio stream is active, False otherwise
        """
        return self.enabled and self.stream is not None and self.stream.active
    
    def __del__(self):
        """Cleanup: stop audio on object deletion."""
        self.stop()
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


# Simple example usage
if __name__ == "__main__":
    import time
    
    # Create engine audio instance
    engine = EngineAudio(
        audio_file="sound/engine.wav",
        idle_rpm=1000,
        redline_rpm=10000,
        min_pitch=0.5,
        max_pitch=2.0,
    )
    
    # Start audio
    engine.start()
    
    if engine.enabled:
        try:
            # Simulate RPM sweep
            print("\nPerforming RPM sweep from idle to redline...")
            for rpm in range(1000, 10000, 100):
                engine.set_rpm(rpm)
                print(f"RPM: {rpm:5d}, Speed: {engine.speed:.3f}")
                time.sleep(0.05)
            
            # Hold at redline
            print("\nHolding at redline...")
            engine.set_rpm(10000)
            time.sleep(2)
            
            # Sweep back down
            print("Sweeping back to idle...")
            for rpm in range(10000, 1000, -100):
                engine.set_rpm(rpm)
                print(f"RPM: {rpm:5d}, Speed: {engine.speed:.3f}")
                time.sleep(0.05)
            
        finally:
            engine.stop()
    
    print("\nDemo complete!")
