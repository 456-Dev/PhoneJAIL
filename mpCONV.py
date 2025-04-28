#!/usr/bin/env python3
"""
Usage: sudo python3 play_mp3_buzzer.py [GPIO_pin] [carrier_frequency_Hz]
Defaults:
  Audio file: pizza.mp3 (in the same folder)
  GPIO_pin: 18
  carrier_frequency_Hz: 2500

This script plays the hardcoded MP3 file (pizza.mp3) through a piezo buzzer connected to a Raspberry Pi.
The buzzer is assumed to have its negative terminal connected to ground and its positive terminal connected to a GPIO pin.
Because passive piezo buzzers cannot faithfully reproduce full audio detail, the script modulates the duty cycle of a fixed-tone PWM
to approximate the MP3's loudness envelope.
Before running the script, install required packages:
  pip install pigpio pydub numpy
and start the pigpio daemon:
  sudo pigpiod
"""

import sys
import time
import numpy as np
import pigpio
from pydub import AudioSegment

def main():
    # Hardcoded audio file (must be in the same folder as this script)
    mp3_file = "pizza.mp3"
    
    # Optional command-line parameters for GPIO pin and carrier frequency
    gpio_pin = int(sys.argv[1]) if len(sys.argv) > 1 else 18      # default GPIO pin 18
    carrier_freq = int(sys.argv[2]) if len(sys.argv) > 2 else 2500  # default 2500 Hz

    # Connect to the pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Make sure to run 'sudo pigpiod'.")
        sys.exit(1)
    
    # Set the PWM frequency for the buzzer output
    pi.set_PWM_frequency(gpio_pin, carrier_freq)

    # Load the MP3 file and downsample it to extract a rough envelope.
    try:
        audio = AudioSegment.from_mp3(mp3_file)
    except Exception as e:
        print("Error loading MP3 file:", e)
        pi.stop()
        sys.exit(1)
    
    # Convert audio to mono and downsample to 1000 Hz for envelope extraction
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(1000)
    
    # Convert the audio samples to a NumPy array
    samples = np.array(audio.get_array_of_samples())
    sample_rate = audio.frame_rate  # should be 1000 Hz now

    # Define a frame length in milliseconds for envelope extraction
    frame_duration_ms = 50  # update roughly every 50 ms
    samples_per_frame = int((sample_rate * frame_duration_ms) / 1000)
    total_frames = len(samples) // samples_per_frame
    max_duty = 255  # PWM duty cycle range: 0 to 255

    print(f"Playing {mp3_file} on GPIO {gpio_pin} at {carrier_freq} Hz carrier ...")
    try:
        for i in range(total_frames):
            # Extract a frame's worth of samples
            frame = samples[i * samples_per_frame : (i+1) * samples_per_frame]
            # Compute the average absolute amplitude (the envelope)
            avg_amp = np.mean(np.abs(frame))
            # Normalize (assuming 16-bit samples; maximum is about 32768)
            duty_cycle = int((avg_amp / 32768) * max_duty)
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
