#!/usr/bin/python3

from periphery import GPIO
import time

# RMIO29 GPIO1_D1 1 * 32 + 3 * 8 + 1 = 57
RESET_Pin = 57

RESET_GPIO = GPIO(RESET_Pin, "out")

try: 
    RESET_GPIO.write(False)
    time.sleep(0.8)
    RESET_GPIO.write(True)
except KeyboardInterrupt:
    RESET_GPIO.write(True)
except IOError:
    print ("Error")

RESET_GPIO.close()
