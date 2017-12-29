import os
import glob
import time
import subprocess
import RPi.GPIO as GPIO
from time import sleep

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    catdata = subprocess.Popen(['cat',device_file], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out,err = catdata.communicate()
    out_decode = out.decode('utf-8')
    lines = out_decode.split('\n')
    return lines

def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c, temp_f

SOUND_PIN = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(SOUND_PIN, GPIO.OUT, initial=GPIO.LOW)
p = GPIO.PWM(SOUND_PIN, 1)
p.start(50)

try:
    while True:
        s = "celsius: {0:.3f}, fahrenheit: {1:.3f}"
        temp = read_temp()
        print(s.format(*temp))
        if float(temp[0]) < 30.0:
            os.system("sudo hub-ctrl -h 0 -P 2 -p 0")
            p.ChangeFrequency(440)
        else:
            os.system("sudo hub-ctrl -h 0 -P 2 -p 1")
            p.ChangeFrequency(1)
        time.sleep(1)
except KeyboardInterrupt:
    p.stop
    GPIO.cleanup()
