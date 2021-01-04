import RPi.GPIO as GPIO
import time

# Pin Definitions
output_pin = 23  # BCM pin 23, BOARD pin 16
output_pin2 = 27 #BCM 27, board 13
output_pin3 = 18#BCN 18, board 12
def main():
    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(output_pin2, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(output_pin3, GPIO.OUT, initial=GPIO.HIGH)
    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH
    try:
        while True:
            time.sleep(1)
            # Toggle the output every second
            print("Outputting {} to pin {}".format(curr_value, output_pin))
            GPIO.output(output_pin, curr_value)
            print("Outputting {} to pin {}".format(curr_value, output_pin2))
            GPIO.output(output_pin2, curr_value)
            print("Outputting {} to pin {}".format(curr_value, output_pin3))
            GPIO.output(output_pin3, curr_value)
            curr_value ^= GPIO.HIGH
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
