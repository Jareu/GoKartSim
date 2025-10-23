"""
Generic looping audio player with adjustable pitch.

Provides an StreamingAudio class that streams an
audio file using sounddevice for real-time playback with seamless looping.
"""

import numpy as np
import sounddevice as sd
import soundfile as sf
from threading import Lock
from pathlib import Path
import os
from typing import Optional


class StreamingAudio:
    """
    Looping audio player with adjustable pitch and volume.
    
    Attributes:
        enabled (bool): Whether audio is currently playing.
        pitch (float): Current pitch multiplier.
        volume (float): Output volume multiplier (1.0 = original loudness).
    """
    
    def __init__(
        self,
        audio_file: str = "sound/engine.wav",
        base_pitch: float = 1.0,
        min_pitch: float = 0.25,
        max_pitch: float = 4.0,
        volume: float = 1.0,
    ):
        """
        Initialize the looping audio system.
        
        Args:
            audio_file: Path to the audio file to loop.
            base_pitch: Initial pitch multiplier (1.0 = original pitch).
            min_pitch: Lower clamp for pitch.
            max_pitch: Upper clamp for pitch.
            volume: Output volume multiplier (0.0 = muted, 1.0 = original).
        """
        self.audio_file = self._resolve_path(audio_file)
        self.base_pitch = base_pitch
        self.min_pitch = max(1e-3, min_pitch)
        self.max_pitch = max(self.min_pitch, max_pitch)
        self.volume = max(0.0, volume)
        
        # Audio state
        self.enabled = False
        self.pitch = self._clamp_pitch(base_pitch)
        self._state_lock = Lock()
        
        # Audio data
        self.data: Optional[np.ndarray] = None
        self.sample_rate: Optional[int] = None
        self.stream: Optional[sd.OutputStream] = None
        self.position = 0.0  # fractional index for interpolation
        
        # Try to load audio file immediately
        self._load_audio()
    
    @staticmethod
    def _resolve_path(audio_file: str) -> str:
        """Resolve audio_file relative to the project if needed."""
        if os.path.isabs(audio_file):
            return audio_file
        
        module_dir = Path(__file__).parent.parent
        resolved_path = module_dir / audio_file
        if resolved_path.exists():
            return str(resolved_path)
        
        if Path(audio_file).exists():
            return audio_file
        
        # Fall back to provided path; start() will warn if it cannot be opened
        return audio_file
    
    def _clamp_pitch(self, value: float) -> float:
        """Clamp pitch within configured limits."""
        return max(self.min_pitch, min(self.max_pitch, value))
    
    def _load_audio(self) -> bool:
        """
        Load the looping audio file.
        
        Returns:
            True if audio loaded successfully, False otherwise.
        """
        try:
            if not os.path.exists(self.audio_file):
                print(f"Warning: Audio file not found: {self.audio_file}")
                return False
            
            self.data, self.sample_rate = sf.read(self.audio_file, dtype="float32")
            
            # Ensure audio is column-major (frames x channels)
            if len(self.data.shape) == 1:
                self.data = self.data.reshape(-1, 1)
            
            print(f"Audio loaded: {self.audio_file}")
            print(f"  Sample rate: {self.sample_rate} Hz")
            print(f"  Duration: {len(self.data) / self.sample_rate:.2f}s")
            print(f"  Channels: {self.data.shape[1]}")
            return True
        except Exception as exc:
            print(f"Error loading audio file {self.audio_file}: {exc}")
            self.data = None
            self.sample_rate = None
            return False
    
    def _callback(self, outdata: np.ndarray, frames: int, time_info, status) -> None:
        """
        Audio stream callback function.
        
        Called by sounddevice to fill the audio buffer. Implements simple pitch
        shifting by varying the pitch and performing linear
        interpolation between samples.
        """
        if status:
            print(f"Audio callback status: {status}")
        
        if self.data is None:
            outdata.fill(0)
            return
        
        with self._state_lock:
            current_pitch = self.pitch
            current_volume = self.volume
        
        data_len = len(self.data)
        engine_chunk = np.zeros((frames, self.data.shape[1]), dtype="float32")
        start_pos = self.position
        
        for i in range(frames):
            sample_pos = start_pos + i * current_pitch
            idx_low = int(sample_pos) % data_len
            idx_high = (idx_low + 1) % data_len
            frac = sample_pos - int(sample_pos)
            engine_chunk[i] = (1 - frac) * self.data[idx_low] + frac * self.data[idx_high]
        
        self.position = (start_pos + current_pitch * frames) % data_len
        
        outdata[:] = engine_chunk * current_volume
    
    def set_pitch(self, pitch: float) -> None:
        """
        Directly set the pitch multiplier.
        
        Args:
            pitch: 1.0 = normal, 0.5 = half pitch, 2.0 = double pitch.
        """
        with self._state_lock:
            self.pitch = self._clamp_pitch(pitch)
    
    def set_volume(self, volume: float) -> None:
        """
        Set the output volume multiplier.
        
        Args:
            volume: Volume multiplier (0.0 = muted, 1.0 = original, >1.0 louder).
        """
        with self._state_lock:
            self.volume = max(0.0, volume)
    
    def start(self) -> bool:
        """
        Start playing the audio loop.
        
        Returns:
            True if audio started successfully, False otherwise.
        """
        if self.data is None:
            print("Error: Audio data not loaded")
            return False
        
        if self.enabled:
            print("Audio already playing")
            return True
        
        try:
            self.position = 0.0
            self.stream = sd.OutputStream(
                callback=self._callback,
                samplerate=self.sample_rate,
                channels=self.data.shape[1],
                blocksize=2048,
            )
            self.stream.start()
            self.enabled = True
            print("Audio stream started")
            return True
        except Exception as exc:
            print(f"Error starting audio stream: {exc}")
            self.stream = None
            self.enabled = False
            return False
    
    def stop(self) -> None:
        """Stop playing the audio loop."""
        if self.stream is not None:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception as exc:
                print(f"Error stopping audio stream: {exc}")
            finally:
                self.stream = None
                self.enabled = False
                print("Audio stream stopped")
    
    def is_playing(self) -> bool:
        """
        Check if audio is currently playing.
        
        Returns:
            True if audio stream is active, False otherwise.
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
