#!/usr/bin/env python3
"""
Usage: sudo python3 play_mp3_buzzer_joystick.py [GPIO_pin] [base_carrier_frequency_Hz]
Defaults:
  Audio file: On.mp3 (in the same folder)
  GPIO_pin: 18
  base_carrier_frequency_Hz: 2500

This script plays the hardcoded MP3 file (On.mp3) via a piezo buzzer using a fixed PWM carrier tone whose duty cycle is modulated based on the MP3's amplitude envelope.
In addition, it reads an analog joystick via an MCP3008 ADC (using gpiozero) so that:
  • The vertical axis (VRy) acts as a volume control.
  • The horizontal axis (VRx) adds a frequency modulation offset.
Before running:
  • Install required packages:
         pip install pigpio pydub numpy gpiozero
  • Start the pigpio daemon: sudo pigpiod
  • Ensure your MCP3008 is wired correctly and the joystick is connected:
         VRx -> MCP3008 channel 0
         VRy -> MCP3008 channel 1
         (Other pins per your wiring)
"""

import sys
import time
import numpy as np
import pigpio
from pydub import AudioSegment
from gpiozero import MCP3008

def map_range(x, in_min, in_max, out_min, out_max):
    """Linearly map x from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main():
    # Use "On.mp3" from the current folder
    mp3_file = "On.mp3"
    
    # Optional command-line parameters:
    # GPIO_pin: output pin for the PWM driving the buzzer (default 18)
    # base_carrier_freq: base PWM frequency in Hz (default 2500)
    gpio_pin = int(sys.argv[1]) if len(sys.argv) > 1 else 18
    base_carrier_freq = int(sys.argv[2]) if len(sys.argv) > 2 else 2500

    # Connect to pigpio daemon for PWM control.
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Make sure to run 'sudo pigpiod'.")
        sys.exit(1)
    
    # Set the initial PWM frequency.
    pi.set_PWM_frequency(gpio_pin, base_carrier_freq)
    
    # Setup joystick via MCP3008 (assumes MCP3008 channels 0 and 1).
    # Channel 0: VRx (horizontal) used for frequency modulation.
    # Channel 1: VRy (vertical) used for volume control.
    joystick_x = MCP3008(channel=0)
    joystick_y = MCP3008(channel=1)
    
    # Load the MP3 file, convert to mono, and downsample to 1000 Hz for envelope extraction.
    try:
        audio = AudioSegment.from_mp3(mp3_file)
    except Exception as e:
        print("Error loading MP3 file:", e)
        pi.stop()
        sys.exit(1)
    
    audio = audio.set_channels(1)
    audio = audio.set_frame_rate(1000)  # downsample for envelope analysis
    
    # Get audio samples as a NumPy array.
    samples = np.array(audio.get_array_of_samples())
    sample_rate = audio.frame_rate  # should be 1000 Hz
    
    # Process MP3 in frames (50 ms each)
    frame_duration_ms = 50
    samples_per_frame = int((sample_rate * frame_duration_ms) / 1000)
    total_frames = len(samples) // samples_per_frame
    max_duty = 255  # Maximum PWM duty cycle value

    # For smoothing the MP3 envelope (exponential moving average)
    alpha = 0.2
    smoothed_duty = 0

    # Frequency modulation range: joystick horizontal can add/subtract up to 500 Hz.
    freq_mod_range = 500

    print(f"Playing {mp3_file} on GPIO {gpio_pin} with base frequency {base_carrier_freq} Hz")
    try:
        for i in range(total_frames):
            # Compute envelope from this MP3 frame.
            frame = samples[i * samples_per_frame : (i+1) * samples_per_frame]
            avg_amp = np.mean(np.abs(frame))
            measured_duty = int((avg_amp / 32768) * max_duty)
            if measured_duty > max_duty:
                measured_duty = max_duty
            # Smooth envelope changes.
            smoothed_duty = int(alpha * measured_duty + (1 - alpha) * smoothed_duty)

            # Read joystick values from MCP3008 (range 0.0 to 1.0).
            js_x = joystick_x.value  # Horizontal axis value.
            js_y = joystick_y.value  # Vertical axis value.

            # Map vertical axis to a volume scale factor.
            # Adjust the mapping as needed: here we assume bottom (0.0) = 0.5x and top (1.0) = 1.0x.
            volume_scale = map_range(js_y, 0, 1, 0.5, 1.0)
            
            # Map horizontal axis to a frequency modulation offset.
            # Center (0.5) gives 0 offset, extremes give ±freq_mod_range.
            freq_mod_offset = map_range(js_x, 0, 1, -freq_mod_range, freq_mod_range)
            
            # Set new PWM frequency based on modulation.
            current_freq = int(base_carrier_freq + freq_mod_offset)
            if current_freq < 1:
                current_freq = 1
            pi.set_PWM_frequency(gpio_pin, current_freq)
            
            # Adjust the PWM duty cycle based on the MP3 envelope and joystick volume.
            final_duty = int(smoothed_duty * volume_scale)
            if final_duty > max_duty:
                final_duty = max_duty
            pi.set_PWM_dutycycle(gpio_pin, final_duty)
            
            # Wait for the next frame.
            time.sleep(frame_duration_ms / 1000.0)
    except KeyboardInterrupt:
        print("Playback interrupted by user.")
    finally:
        # Clean up
        pi.set_PWM_dutycycle(gpio_pin, 0)
        pi.stop()
        print("Playback finished.")

if __name__ == '__main__':
    main() 