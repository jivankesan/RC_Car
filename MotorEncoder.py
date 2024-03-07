import RPi.GPIO as GPIO
import time

# Constants
GPIO_PIN_ENCODER_A = 22  # Change to your GPIO pin number
GPIO_PIN_ENCODER_B = 24  # Change to your GPIO pin number
PULSES_PER_REVOLUTION = 11*280  # Adjust to match your encoder's specification
WHEEL_CIRCUMFERENCE = 0.300 # Meter 

# Variables
counter_a = 0
counter_b = 0

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_PIN_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_PIN_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Callbacks
def encoder_callback_a(channel):
    global counter_a
    counter_a += 1

def encoder_callback_b(channel):
    global counter_b
    counter_b += 1

# Interrupts
GPIO.add_event_detect(GPIO_PIN_ENCODER_A, GPIO.RISING, callback=encoder_callback_a)
GPIO.add_event_detect(GPIO_PIN_ENCODER_B, GPIO.RISING, callback=encoder_callback_b)

try:
    while True:
        time.sleep(1)  # Main loop does nothing but wait
        # Calculate distance traveled
        revolutions_a = counter_a / PULSES_PER_REVOLUTION
        distance_a = revolutions_a * WHEEL_CIRCUMFERENCE
        revolutions_b = counter_b / PULSES_PER_REVOLUTION
        distance_b = revolutions_b * WHEEL_CIRCUMFERENCE
        print(f"Motor A: {distance_a} meters, Motor B: {distance_b} meters")
except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit
