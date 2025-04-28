#!/usr/bin/env python3
"""
Usage: sudo python3 play_mp3_buzzer.py [GPIO_pin] [carrier_frequency_Hz]
Defaults:
  Audio file: pizza.mp3 (in the same folder)
  GPIO_pin: 18
  carrier_frequency_Hz: 2500

This script plays the hardcoded MP3 file (pizza.mp3) through a piezo buzzer
connected to a Raspberry Pi. It uses a fixed PWM carrier tone and modulates its duty
cycle based on the audio amplitude envelope. An exponential moving average is applied
to smooth abrupt amplitude changes, resulting in a slightly more natural output.
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
import machine
import utime

# Configure ADC objects for the joystick axes:
# VRx (GP27 / ADC1) controls the frequency.
adc_freq = machine.ADC(27)
# VRy (GP26 / ADC0) controls the volume.
adc_vol = machine.ADC(26)

# Configure the joystick button if needed (SW on GP16).
joystick_button = machine.Pin(16, machine.Pin.IN, machine.Pin.PULL_UP)

# Configure the PWM output for the piezo buzzer.
# Change this pin if your buzzer is connected elsewhere.
buzzer_pin = machine.Pin(15)
buzzer_pwm = machine.PWM(buzzer_pin)

# Mapping helper function: linearly maps x from [in_min, in_max] to [out_min, out_max].
def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def main():
    # Hardcoded audio file (should be in the same folder as this script)
    mp3_file = "pizza.mp3"
    
    # Optional command-line parameters: GPIO pin and carrier frequency
    gpio_pin = int(sys.argv[1]) if len(sys.argv) > 1 else 18      # default GPIO pin 18
    carrier_freq = int(sys.argv[2]) if len(sys.argv) > 2 else 2500  # default 2500 Hz

    # Connect to the pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Make sure to run 'sudo pigpiod'.")
        sys.exit(1)
    
    # Set the PWM carrier frequency for the buzzer output
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
    sample_rate = audio.frame_rate  # should be 1000 Hz

    # Define a frame length in milliseconds (update roughly every 50 ms)
    frame_duration_ms = 50
    samples_per_frame = int((sample_rate * frame_duration_ms) / 1000)
    total_frames = len(samples) // samples_per_frame
    max_duty = 255  # PWM duty cycle range: 0 to 255

    # Smoothing parameters for exponential moving average:
    alpha = 0.2  # Smoothing factor (0 < alpha <= 1). Lower values yield smoother transitions.
    smoothed_duty = 0  # Initialize smoothed duty cycle

    print(f"Playing {mp3_file} on GPIO {gpio_pin} at {carrier_freq} Hz carrier ...")
    try:
        for i in range(total_frames):
            # Extract a frame's worth of samples
            frame = samples[i * samples_per_frame : (i+1) * samples_per_frame]
            # Compute the average absolute amplitude (the envelope)
            avg_amp = np.mean(np.abs(frame))
            # Normalize the amplitude to determine duty cycle (assuming 16-bit samples; max ~32768)
            measured_duty = int((avg_amp / 32768) * max_duty)
            if measured_duty > max_duty:
                measured_duty = max_duty

            # Apply exponential moving average to smooth the duty cycle transitions
            smoothed_duty = int(alpha * measured_duty + (1 - alpha) * smoothed_duty)

            # Update the PWM duty cycle accordingly
            pi.set_PWM_dutycycle(gpio_pin, smoothed_duty)
            time.sleep(frame_duration_ms / 1000.0)
    except KeyboardInterrupt:
        print("Playback interrupted by user.")
    finally:
        # Turn off the PWM signal and release resources
        pi.set_PWM_dutycycle(gpio_pin, 0)
        pi.stop()
        print("Playback finished.")

    # On the Pico, ADC readings (read_u16) return a value in the range 0 to 65535.
    # For PWM in MicroPython, duty is set using duty_u16 (0 to 65535).
    while True:
        # Read the ADC values.
        freq_val = adc_freq.read_u16()  # value from 0 to 65535 controlling frequency
        vol_val = adc_vol.read_u16()    # value from 0 to 65535 controlling volume

        # Map the frequency ADC reading to a frequency range.
        freq = map_range(freq_val, 0, 65535, 100, 3000)
        # Map the volume ADC reading directly as duty cycle.
        duty = map_range(vol_val, 0, 65535, 0, 65535)

        # Set the new frequency and duty cycle on the buzzer.
        buzzer_pwm.freq(freq)
        buzzer_pwm.duty_u16(duty)
        
        # Optional: read the joystick button.
        # If the button is pressed (value goes low), you could add extra functionality.
        # if not joystick_button.value():
        #     print("Joystick button pressed!")
        
        # Update at a 50 ms interval.
        utime.sleep(0.05)

if __name__ == '__main__':
    main() 