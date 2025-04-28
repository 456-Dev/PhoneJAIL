#!/usr/bin/env python3
"""
Usage: sudo python3 play_mp3_buzzer.py <mp3_file> [GPIO_pin] [carrier_frequency_Hz]
Defaults:
  GPIO_pin: 18
  carrier_frequency_Hz: 2500

This script plays an MP3 file through a piezo buzzer connected to a Raspberry Pi.
The buzzer is assumed to have its negative terminal connected to ground and its positive terminal
connected to a GPIO pin. Because passive piezo buzzers cannot reproduce full audio detail,
this script reproduces the loudness envelope of the MP3 by modulating the duty cycle of a fixed-tone PWM.
Before running the script, install the required packages with:
  pip install pigpio pydub numpy
and start the pigpio daemon using:
  sudo pigpiod
"""

import sys
import time
import numpy as np
import pigpio
from pydub import AudioSegment

def main():
    if len(sys.argv) < 2:
        print("Usage: sudo python3 play_mp3_buzzer.py <mp3_file> [GPIO_pin] [carrier_frequency_Hz]")
        sys.exit(1)
    
    mp3_file = sys.argv[1]
    gpio_pin = int(sys.argv[2]) if len(sys.argv) > 2 else 18  # default GPIO pin 18
    carrier_freq = int(sys.argv[3]) if len(sys.argv) > 3 else 2500  # default 2500 Hz (adjust for your buzzer)

    # Connect to the pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Make sure to run 'sudo pigpiod'.")
        sys.exit(1)
    
    # Set the PWM frequency for the buzzer output
    pi.set_PWM_frequency(gpio_pin, carrier_freq)

    # Load the MP3 file and downsample it to extract a rough envelope.
    # Convert to mono and lower the sample rate to drive the PWM update loop.
    try:
        audio = AudioSegment.from_mp3(mp3_file)
    except Exception as e:
        print("Error loading MP3 file:", e)
        pi.stop()
        sys.exit(1)
    
    audio = audio.set_channels(1)
    # Downsample to 1000 Hz for envelope extraction (1000 samples per second)
    audio = audio.set_frame_rate(1000)

    # Convert the audio samples to a NumPy array
    samples = np.array(audio.get_array_of_samples())
    sample_rate = audio.frame_rate  # should be 1000 Hz now

    # Define a frame length in milliseconds (adjust resolution and smoothness)
    frame_duration_ms = 50  # update roughly every 50 ms
    samples_per_frame = int((sample_rate * frame_duration_ms) / 1000)
    total_frames = len(samples) // samples_per_frame
    max_duty = 255  # PWM duty cycle value ranges from 0 to 255

    print(f"Playing {mp3_file} on GPIO {gpio_pin} at {carrier_freq} Hz carrier ...")
    try:
        for i in range(total_frames):
            # Extract a frame's worth of samples
            frame = samples[i * samples_per_frame : (i+1) * samples_per_frame]
            # Compute the average absolute amplitude (the envelope)
            avg_amp = np.mean(np.abs(frame))
            # Normalize (assuming 16-bit samples; maximum is about 32768)
            duty_cycle = int((avg_amp / 32768) * max_duty)
            # Clamp the duty cycle to the maximum
            if duty_cycle > max_duty:
                duty_cycle = max_duty
            # Set the PWM duty cycle accordingly
            pi.set_PWM_dutycycle(gpio_pin, duty_cycle)
            # Wait for the frame duration
            time.sleep(frame_duration_ms / 1000.0)
    except KeyboardInterrupt:
        print("Playback interrupted by user.")
    finally:
        # Turn off the PWM signal and release resources
        pi.set_PWM_dutycycle(gpio_pin, 0)
        pi.stop()
        print("Playback finished.")

if __name__ == '__main__':
    main()
