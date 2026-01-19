#!/usr/bin/env python3
from .basic import _Basic_class
""" import time
import threading
import pyaudio
import os
import struct
import math
from .utils import enable_speaker, disable_speaker """

class Music(_Basic_class):
    """Play music, sound affect and note control"""

    FORMAT = None
    CHANNELS = 1
    RATE = 44100

    KEY_G_MAJOR = 1
    KEY_D_MAJOR = 2
    KEY_A_MAJOR = 3
    KEY_E_MAJOR = 4
    KEY_B_MAJOR = 5
    KEY_F_SHARP_MAJOR = 6
    KEY_C_SHARP_MAJOR = 7

    KEY_F_MAJOR = -1
    KEY_B_FLAT_MAJOR = -2
    KEY_E_FLAT_MAJOR = -3
    KEY_A_FLAT_MAJOR = -4
    KEY_D_FLAT_MAJOR = -5
    KEY_G_FLAT_MAJOR = -6
    KEY_C_FLAT_MAJOR = -7

    KEY_SIGNATURE_SHARP = 1
    KEY_SIGNATURE_FLAT = -1

    WHOLE_NOTE = 1
    HALF_NOTE = 1/2
    QUARTER_NOTE = 1/4
    EIGHTH_NOTE = 1/8
    SIXTEENTH_NOTE = 1/16

    NOTE_BASE_FREQ = 440
    """Base note frequency for calculation (A4)"""
    NOTE_BASE_INDEX = 69
    """Base note index for calculation (A4) MIDI compatible"""

    NOTES = [
        None,  None, None,  None, None, None,  None, None,  None, None,  None, None,
        None,  None, None,  None, None, None,  None, None,  None, "A0", "A#0", "B0",
        "C1", "C#1", "D1", "D#1", "E1", "F1", "F#1", "G1", "G#1", "A1", "A#1", "B1",
        "C2", "C#2", "D2", "D#2", "E2", "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2",
        "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3",
        "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
        "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
        "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
        "C7", "C#7", "D7", "D#7", "E7", "F7", "F#7", "G7", "G#7", "A7", "A#7", "B7",
        "C8"]
    """Notes name, MIDI compatible"""

    def __init__(self):
        pass
    def time_signature(self, top: int = None, bottom: int = None):
        pass

    def key_signature(self, key: int = None):
        """
        Set/get key signature

        :param key: key signature use KEY_XX_MAJOR or String "#", "##", or "bbb", "bbbb"
        :type key: int/str
        :return: key signature
        :rtype: int
        """
        pass

    def tempo(self, tempo=None, note_value=QUARTER_NOTE):
        """
        Set/get tempo beat per minute(bpm)

        :param tempo: tempo
        :type tempo: float
        :param note_value: note value(1, 1/2, Music.HALF_NOTE, etc)
        :return: tempo
        :rtype: int
        """
        pass

    def beat(self, beat):
        """
        Calculate beat delay in seconds from tempo

        :param beat: beat index
        :type beat: float
        :return: beat delay
        :rtype: float
        """
        return 0.0

    def note(self, note, natural=False):
        """
        Get frequency of a note

        :param note_name: note name(See NOTES)
        :type note_name: string
        :param natural: if natural note
        :type natural: bool
        :return: frequency of note
        :rtype: float
        """
        return 0.0

    def sound_play(self, filename, volume=None):
        """
        Play sound effect file

        :param filename: sound effect file name
        :type filename: str
        """
        pass

    def sound_play_threading(self, filename, volume=None):
        """
        Play sound effect in thread(in the background)

        :param filename: sound effect file name
        :type filename: str
        :param volume: volume 0-100, leave empty will not change volume
        :type volume: int
        """
        pass
    def music_play(self, filename, loops=1, start=0.0, volume=None):
        """
        Play music file

        :param filename: sound file name
        :type filename: str
        :param loops: number of loops, 0:loop forever, 1:play once, 2:play twice, ...
        :type loops: int
        :param start: start time in seconds
        :type start: float
        :param volume: volume 0-100, leave empty will not change volume
        :type volume: int
        """
        pass

    def music_set_volume(self, value):
        """
        Set music volume

        :param value: volume 0-100
        :type value: int
        """
        pass
    def music_stop(self):
        """Stop music"""
        pass

    def music_pause(self):
        """Pause music"""
        pass

    def music_resume(self):
        """Resume music"""
        pass

    def music_unpause(self):
        """Unpause music(resume music)"""
        pass

    def sound_length(self, filename):
        """
        Get sound effect length in seconds

        :param filename: sound effect file name
        :type filename: str
        :return: length in seconds
        :rtype: float
        """
        return 0.0

    def get_tone_data(self, freq: float, duration: float):
        """
        Get tone data for playing

        :param freq: frequency
        :type freq: float
        :param duration: duration in seconds
        :type duration: float
        :return: tone data
        :rtype: list
        """
        """
        Credit to: Aditya Shankar & Gringo Suave https://stackoverflow.com/a/53231212/14827323
        """
        return b""

    def play_tone_for(self, freq, duration):
        """
        Play tone for duration seconds

        :param freq: frequency, you can use NOTES to get frequency
        :type freq: float
        :param duration: duration in seconds
        :type duration: float
        """
        """
        Credit to: Aditya Shankar & Gringo Suave https://stackoverflow.com/a/53231212/14827323
        """
        pass