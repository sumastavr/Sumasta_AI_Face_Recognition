import sensor
import image
import lcd
import KPU as kpu
import time
from Maix import FPIOA, GPIO
import gc
from fpioa_manager import fm
from board import board_info
import utime

from machine import Timer,PWM
from board import board_info

from msa301 import MSA301
from machine import I2C

from Maix import I2S
import audio

# Define servo class
class Servo:
  def __init__(self, pwm, dir=50, duty_min=4.25, duty_max=10.5):
      self.value = dir
      self.pwm = pwm
      self.duty_min = duty_min
      self.duty_max = duty_max
      self.duty_range = duty_max -duty_min
      self.enable(True)
      self.pwm.duty(self.value/100*self.duty_range+self.duty_min)

  def enable(self, en):
      if en:
          self.pwm.enable()
      else:
          self.pwm.disable()

  def dir(self, percentage):
      if percentage > 1000:
          percentage = 1000
      elif percentage < 0:
          percentage = 0
      self.pwm.duty(percentage/1000*self.duty_range+self.duty_min)

  def drive(self, inc):
      self.value += inc
      if self.value > 100:
          self.value = 100
      elif self.value < 0:
          self.value = 0
      self.pwm.duty(self.value/100*self.duty_range+self.duty_min)


SSID = ["Trulifi-6014/6016-TestCenter","WiFi 755"]
PASW = ["Signify2021!","printedcircuitboard"]

IP_ADD="NO NETWORK"
WIFI_CONNECTED=False

# Wifi initialization logic
def enable_espat():
  from network_espat import wifi
  if wifi.isconnected() == False:
      for i in range(2):
          try:
              # Running within 3 seconds of power-up can cause an SD load error
              # wifi.reset(is_hard=False)
              wifi.reset()
              print('try AT connect wifi...')
              wifi.connect(SSID[i], PASW[i])
              if wifi.isconnected():
                  test=wifi.ifconfig()
                  global IP_ADD
                  IP_ADD=str(test[0])
                  print("IP:",IP_ADD)
                  global WIFI_CONNECTED
                  WIFI_CONNECTED=True
                  break
          except Exception as e:
              print(e)
  print('network state:', wifi.isconnected(), wifi.ifconfig())
  return wifi.isconnected()

try:
  import usocket as socket
except:
  import socket

TestHttps = False

def sendHTTPCommand(command,use_stream=True):

  s = socket.socket()
  s.settimeout(0.1)
  host = "192.168.0.112"
  if TestHttps:
      ai = socket.getaddrinfo(host, 443)
  else:
      ai = socket.getaddrinfo(host, 80)
  print("Address infos:", ai)
  addr = ai[0][-1]
  for i in range(5):
      try:
          print("Connect address:", addr)
          s.connect(addr)

          str_command=""
          str_command+="POST " + command + " / HTTP/1.1\r\n\r\n"
          s.write(str_command)

          """
          if TestHttps: # ssl
              try:
                  import ussl as ssl
              except:
                  import ssl
              tmp = ssl.wrapsocket(s, server_hostname=host)
              #tmp.write(b"GET / HTTP/1.1\r\n\r\n")
          else:
              #s.write(b"GET / HTTP/1.1\r\n\r\n")
              #print("post")
              str_command=""
              str_command+="POST " + command + " / HTTP/1.1\r\n\r\n"
              s.write(str_command)
          """

          data = (s.readline('\r\n'))
          print(data)
          #with open('test.txt', 'wb') as f:
          #    f.write(data)

      except Exception as e:
        print(e)

  s.close()

L_S = b'\x00\x1F\x7F\xFF\xF0\xF0\xFF\x7F\x1F\x00\x00\xFF\xFF\xFF\x00\x00\x00\xFF\xFF\xFF\x00\x00\xFC\xFE\xFF\x0F\x0F\xFF\xFE\xFC\x00\x00'
L_U = b'\x00\xF0\xF0\xF0\xF0\xF0\xF0\xF0\xF0\xF8\x7C\x3F\x1F\x07\x00\x00\x00\x0F\x0F\x0F\x0F\x0F\x0F\x0F\x0F\x0F\x0F\xFF\xFF\xFF\x00\x00'
L_M = b'\x00\x07\x1F\x3F\x7F\xFB\xF3\xF3\xF3\xF3\xF3\xF3\xF3\xF3\x00\x00\x00\xFF\xFF\xFF\xCF\xCF\xCF\xCF\xCF\xCF\xCF\xCF\xCF\xCF\x00\x00'
L_A = b'\x00\x07\x1F\x3F\x7C\xF8\xF0\xFF\xFF\xFF\xF0\xF0\xF0\xF0\x00\x00\x00\xE0\xF8\xFC\x3E\x1F\x0F\xFF\xFF\xFF\x0F\x0F\x0F\x0F\x00\x00'
L_T = b'\x00\x03\x03\x03\x03\x03\xFF\xFF\xFF\x03\x03\x03\x03\x03\x00\x00\x00\xC0\xC0\xC0\xC0\xC0\xFF\xFF\xFF\xC0\xC0\xC0\xC0\xC0\x00\x00'

L_START_X=100
Y_POS=10
SPACING = 1

STATUS_BANNER="Tracking Face"
BANNER_COLOR=(255,0,0)

def drawLogo():
  a = img.draw_font(L_START_X, Y_POS, 16, 16, L_S, scale=1, color=(255, 255, 255))
  a = img.draw_font(L_START_X+(16*1)+SPACING*1, Y_POS, 16, 16, L_U, scale=1, color=(255, 255, 255))
  a = img.draw_font(L_START_X+(16*2)+SPACING*2, Y_POS, 16, 16, L_M, scale=1, color=(255, 255, 255))
  a = img.draw_font(L_START_X+(16*3)+SPACING*3, Y_POS, 16, 16, L_A, scale=1, color=(255, 255, 255))
  a = img.draw_font(L_START_X+(16*4)+SPACING*4, Y_POS, 16, 16, L_S, scale=1, color=(255, 255, 255))
  a = img.draw_font(L_START_X+(16*5)+SPACING*5, Y_POS, 16, 16, L_T, scale=1, color=(255, 0, 0))
  a = img.draw_font(L_START_X+(16*6)+SPACING*6, Y_POS, 16, 16, L_A, scale=1, color=(255, 255, 255))


def drawDashboard():
  accel_value_Y=accel.acceleration[1]

  a = img.draw_string(1, 100, ("Motor Pos: %2.1f" % currentDir), scale=1, color=(0,255,50))
  a = img.draw_string(1, 120, ("PID Error: %2.1f" % PID_ERROR), scale=1, color=(255,0,50))
  a = img.draw_string(1, 140, ("Accel Y: %2.1f" % accel_value_Y), scale=1, color=(126,0,255))

  if WIFI_CONNECTED:
      img.draw_string(1, 162, ("WiFi Connected"), scale=1, color=(255,255,0))
  else:
      img.draw_string(1, 162, ("WiFi Disconnected"), scale=1, color=(255,255,0))

  IP_STR="IP: "+str(IP_ADD)
  img.draw_string(1, 172, IP_STR, scale=1, color=(255,255,0))

  a = img.draw_rectangle(1,111,80,9,color=(255,255,255))
  a = img.draw_rectangle(1,131,80,9,color=(255,255,255))
  a = img.draw_rectangle(1,151,80,9,color=(255,255,255))
  #img.draw_rectangle(1,171,80,9,color=(255,255,255))
  #img.draw_rectangle(1,191,80,9,color=(255,255,255))

  x_currentDir = int (currentDir * 80 / 1000)
  img.draw_circle(x_currentDir,115,4,fill=True,color=(0,255,50))

  x_pid_error = int (PID_ERROR * 80 / 50)
  a = img.draw_rectangle((2,132,x_pid_error,7),fill=True,color=(255,0,50))

  if accel_value_Y<0:
      accel_value_Y*=-1
  x_accel_y = int (accel_value_Y * 80 / 5)
  a = img.draw_rectangle((2,152,x_accel_y,7),fill=True,color=(126,0,255))


  str_day_runtime="Runtime "+str(t1[2])+" Day"
  img.draw_string(1, 183, str_day_runtime, scale=1, color=(255,255,255))

  str_SMH_clock=str(t1[3])+" : "+str(t1[4])+" : "+str(t1[5])
  img.draw_string(15, 193, str_SMH_clock, scale=1, color=(255,255,255))

  img.draw_rectangle((0,216,240,24),fill=True,color=(0,0,0))
  img.draw_rectangle((0,206,40,10),fill=True,color=(0,0,0))
  img.draw_string(1, 206, "Status: ", scale=1, color=(0,255,255))
  img.draw_string(1, 216, STATUS_BANNER, scale=2, color=(0,255,255))

  if ATTENDANCE_STATUS:
      img.draw_circle(15,15,6,fill=True,color=(0,255,0))
  else:
      img.draw_circle(15,15,6,fill=True,color=(255,0,0))

  img.draw_string(280, 5, '%.2f' % fps, scale=1.5, color=(255,255,255))
  img.draw_string(290, 20, "FPS", scale=1, color=(255,255,255))

  remainingShut=utime.ticks_diff(utime.ticks_ms(),ATTENDANCE_COUNTER)
  remainingShut=ATTENDANCE_TIMEOUT-remainingShut
  remainingShut/=1000

  #if remainingShut<30 and remainingShut>28:
    #player = audio.Audio(path="/sd/doorbell2.wav")
    #player.play()
    #player.finish()

  img.draw_string(10, 30, '%.1f' % remainingShut, scale=1.5, color=(255,255,255))

# IRQ routine for starting face calibration / storing routine
def set_key_state(*_):
  global start_processing
  start_processing = True
  #global counter_face_training
  #counter_face_training = counter_face_training + 1
  utime.sleep_ms(BOUNCE_PROTECTION)

# IRQ routine for shutting down manually
def shutdown(*_):
  global ATTENDANCE_STATUS
  ATTENDANCE_STATUS=False
  global REMOTE_CTRL_WIFI
  REMOTE_CTRL_WIFI[0]=False
  print("Timeout: no recognized faced")
  sendHTTPCommand(str(REMOTE_CTRL_WIFI[0]))
  utime.sleep_ms(BOUNCE_PROTECTION)

# Define the RGB LED indicators connected to the board
IO_LED_RED = 14
IO_LED_GREEN = 13
IO_LED_BLUE = 12

# Register the GPIO to internal MUX GPIO
fm.register(IO_LED_RED, fm.fpioa.GPIO0)
fm.register(IO_LED_GREEN, fm.fpioa.GPIO1)
fm.register(IO_LED_BLUE, fm.fpioa.GPIO2)

# Config GPIO to output
LED_RED=GPIO(GPIO.GPIO0, GPIO.OUT)
LED_GREEN=GPIO(GPIO.GPIO1, GPIO.OUT)
LED_BLUE=GPIO(GPIO.GPIO2, GPIO.OUT)

# Set initial value of the LEDs 1 means OFF
LED_RED.value(1)
LED_GREEN.value(1)
LED_BLUE.value(1)

# Load the neural network model from the flash, location must be made according to the programmed
task_fd = kpu.load(0x300000)
task_ld = kpu.load(0x400000)
task_fe = kpu.load(0x500000)

# initialize utime clock
clock = time.clock()

# define the push buttons IO
IO_BUTTON_CENTER = 15
IO_BUTTON_LEFT = 16
IO_BUTTON_RIGHT = 17

# Register the button to input pins
fm.register(IO_BUTTON_CENTER, fm.fpioa.GPIOHS2)
fm.register(IO_BUTTON_LEFT, fm.fpioa.GPIOHS3)
key_gpio = GPIO(GPIO.GPIOHS2, GPIO.IN)
key_shut = GPIO(GPIO.GPIOHS3, GPIO.IN)
start_processing = False
init_processing = True

# bouncing protection for the switch
BOUNCE_PROTECTION = 200
counter_face_training = 1

# initialize interrupt routine for the GPIO
key_gpio.irq(set_key_state, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT)
key_shut.irq(shutdown, GPIO.IRQ_FALLING, GPIO.WAKEUP_NOT_SUPPORT)

# initialize LCD speed and registers
lcd.init(freq=15000000)
lcd.direction(0x60)
#lcd.init(type=2)
#lcd.rotation(2)

img_init=image.Image()

img_init.draw_string(0,0,"Initialize Camera...",scale=1.5,color=(255,255,255))
lcd.display(img_init)

# initialize Camera sensor and its registry
sensor.reset(freq=24000000,set_regs=True,dual_buff=True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_contrast(2)
sensor.set_saturation(-2)
#sensor.set_auto_gain(True)
#sensor.set_hmirror(1)
#sensor.set_brightness(0)
#sensor.set_jb_quality(100)
sensor.set_vflip(1)
sensor.run(1)

# Accelerometer sensor initialization
i2c = I2C(I2C.I2C1, freq=100*1000, sda=31, scl=30)
dev_list = i2c.scan()

img_init.draw_string(300,0,"OK",scale=1.5,color=(0,255,0))
img_init.draw_string(0,15,"Initialize Accelerometer...",scale=1.5,color=(255,255,255))
lcd.display(img_init)

# generate instance and enable tap detection.
accel=MSA301(i2c)
accel.enable_tap_detection()

img_init.draw_string(300,15,"OK",scale=1.5,color=(0,255,0))
img_init.draw_string(0,30,"Initialize CNN Model Buffer...",scale=1.5,color=(255,255,255))
lcd.display(img_init)

anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437,
        6.92275, 6.718375, 9.01025)  # anchor for face detect

dst_point = [(44, 59), (84, 59), (64, 82), (47, 105),
           (81, 105)]  # standard face key point position

a = kpu.init_yolo2(task_fd, 0.5, 0.3, 5, anchor)

img_lcd = image.Image()
img_face = image.Image(size=(128, 128)) # create a blank image with 128 x 126 in size
a = img_face.pix_to_ai() # important steps to convert RGB565 to RGB888 that is needed by KPU

record_ftr = []
record_ftrs = []

img_init.draw_string(300,30,"OK",scale=1.5,color=(0,255,0))
img_init.draw_string(0,45,"Initialize Servo Motor...",scale=1.5,color=(255,255,255))
lcd.display(img_init)

# Servo motor initialization
init_pitch = 500
tim0 = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
pitch_pwm = PWM(tim0, freq=50, duty=0, pin=11) # 50 Hz frequency, period 20ms, on HPIO 11
pitch = Servo(pitch_pwm, dir=init_pitch)
pitch.dir(500)

# Read memory heap size
import Maix
print('Current: ', Maix.utils.gc_heap_size())

img_init.draw_string(300,45,"OK",scale=1.5,color=(0,255,0))
img_init.draw_string(0,60,"Initialize WiFi Connection...",scale=1.5,color=(255,255,255))
lcd.display(img_init)

# enable WiFi connection to the router.
# after several trials it will either return true or false
if enable_espat():
  img_init.draw_string(0,75,"WiFi CONNECTED",scale=1.5,color=(0,255,0))
  lcd.display(img_init)
else:
  img_init.draw_string(0,75,"NO Wifi Connection",scale=1.5,color=(255,0,0))
  lcd.display(img_init)

utime.sleep(2)

# open audio PA
"""
fm.register(32, fm.fpioa.GPIO4, force=True)
AUDIO_EN=GPIO(GPIO.GPIO4, GPIO.OUT)
# Set initial value of the LEDs 1 means OFF
AUDIO_EN.value(1)

# register i2s(i2s0) pin
fm.register(34, fm.fpioa.I2S0_OUT_D1, force=True)
fm.register(35, fm.fpioa.I2S0_SCLK, force=True)
fm.register(33, fm.fpioa.I2S0_WS, force=True)

# init i2s(i2s0)
wav_dev = I2S(I2S.DEVICE_0)

# init audio
player = audio.Audio(path="/sd/doorbell2.wav")
player.volume(10)

# read audio info
wav_info = player.play_process(wav_dev)
print("wav file head information: ", wav_info)

# config i2s according to audio info
wav_dev.channel_config(wav_dev.CHANNEL_1, I2S.TRANSMITTER, resolution=I2S.RESOLUTION_16_BIT,
                       cycles=I2S.SCLK_CYCLES_32, align_mode=I2S.RIGHT_JUSTIFYING_MODE)
wav_dev.set_sample_rate(wav_info[1])
"""

# GLOBAL VARIABLES FOR LOGIC AND FUNCTIONALITIES OF THE OVERALL SYSTEM

ACCURACY = 85 # Threshold of the face recognition detection

AUTH_STARTED = False # boolean to
AUTH_TIME_OUT = 3000
AUTH_TIMER = utime.ticks_ms()
AUTH_COUNTER = 0
AUTH_N_ACCEPTANCE = 5

currentDir=init_pitch # set initial current position of the servo identical to init-pitch

### PID variables definitions ##

# tested suitable for >=35Fps
#C_PID_P=0.9
#C_PID_I=0.0001
#C_PID_D=0.05

# tested suitable for >=32Fps smoother moving
C_PID_P=0.75 # Proportional gain constant
C_PID_I=0.00015 # Integral Gain constant
C_PID_D=0.5 # Derivative Gain constant

# Variables definition that holds the PID Error
PID_ERROR=0 # Current PID error
TOTAL_ERROR=0 # Total accumulated / Integrated error
PAST_ERROR=0 # Error value on the previous one iteration of the loop

# PID set point / Target value
TARGET=160 # Target set point of the control loop. 160 is the center of the LCD width/camera (320/2)
TARGET_MARGIN=3 # Target margin of the set point, plus and minus, within this boundary PID error = 0

FACE_CENTERED=False # global boolean that holds the status whether face is centered or tracking has reached set point

# Variable that holds value of face detection at the previous loop
PREV_FACE_LOC=0;
PREV_FACE_SIZE=0;

N_FACE_DETECTED=1; # variable holds the number of currently detected face. if multiple face is detected tracking logic will be inactivated

# Variables and counters that holds and trigger whether it is to return to reset position or not.
COUNTER_RETURN_POS=utime.ticks_ms()
TIMEOUT_RETURN_POS=5000
BACK_POS=True

# Variables that holds status of connected (wireless) switch
REMOTE_CTRL_WIFI=[False,False,False] # Boolean array keeping the status of switches/relay
FACE_AUTHED=False # holds the status whether face has been succesfully authenticated

# Variables and counters that trigers to switch off devices when it timed-out
ATTENDANCE_COUNTER=utime.ticks_ms()
ATTENDANCE_TIMEOUT=5*60*1000
ATTENDANCE_STATUS=False

CONTINUOUS_FALSE_COUNTER=100 # number of maximum continuous false detection until certain trigger is activated i.e recording

fps=0.0 # fps variable holder

t1 = time.localtime(int(utime.ticks_ms()/1000))
SECONDS_TICKER=utime.ticks_ms()

# Variables and counters for triggering picture capture
COUNTER_SHOOT=utime.ticks_ms()
TIMEOUT_SHOOT=10000
FILE_COUNTER=0

# Total number of trained face
TOTAL_TRAINED_FACE = 20

while (1):

  img = sensor.snapshot()

  # If conditions that trigger every second to update clock and other things
  if utime.ticks_diff(utime.ticks_ms(),SECONDS_TICKER)>1000:
      print("%2.1f fps" % fps)
      t1 = time.localtime(int(utime.ticks_ms()/1000))
      SECONDS_TICKER=utime.ticks_ms()

  """
  #If conditions that trigger every second to update clock and other things
  if utime.ticks_diff(utime.ticks_ms(),COUNTER_SHOOT)>TIMEOUT_SHOOT:
  # inactive function to save current camera capture to SD card
    path = "/sd/camera-" + str(FILE_COUNTER) + ".jpg"
    #time.sleep(1)
    img.save(path)
    FILE_COUNTER += 1
    print("save photo")

    with open("/sd/photocounter.txt", "w") as f:
        f.write(FILE_COUNTER)

    COUNTER_SHOOT=utime.ticks_ms()
  """

  clock.tick()
  code = kpu.run_yolo2(task_fd, img) # Run Face detection model

  #print("%f %f %f" % accel.acceleration)
  #accel.acceleration
  #if accel.tapped:
  #    print("Tapped")
  #    time.sleep(0.1)

  drawLogo() # call function to draw logo
  drawDashboard() # call function to draw dashboards on the left side of the screen

  # If conditions that trigger when no recognized face was detected and timeout
  if utime.ticks_diff(utime.ticks_ms(),ATTENDANCE_COUNTER)>ATTENDANCE_TIMEOUT and ATTENDANCE_STATUS==True:
      ATTENDANCE_STATUS=False
      REMOTE_CTRL_WIFI[0]=False
      TOTAL_ERROR=0
      print("Timeout: no recognized faced")
      sendHTTPCommand(str(REMOTE_CTRL_WIFI[0])) # Send http command to disable wireless switch

  # If multiple face detected, change status banner, tracking will be disabled
  if N_FACE_DETECTED>1:
      STATUS_BANNER="Mult Face, Stop Tracking"

  # Initial condition check before running PID loop, it only runs when only 1 face is detected
  if N_FACE_DETECTED == 1:
      STATUS_BANNER="Tracking Face"
      # check if target is smaller or higher than setpoint (160), this is just to keep PID error to be always positive number
      if TARGET<160:
          PID_ERROR = 160-TARGET # get current PID error
          if PID_ERROR>50: # optional PID error cap, to limit overshoot in movement in case error is suddenly to big
              PID_ERROR=50
          # Another check to see whether the error is within the target margin as specified above, if so, force the PID error to 0 and set face centered
          if PID_ERROR<TARGET_MARGIN and PID_ERROR >-TARGET_MARGIN:
              PID_ERROR=0
              FACE_CENTERED=True
              #print(TARGET,"CAPTURE LEFT")
          else: # if not within the target margin, then implement the PID gains and put the value to currentdir variable
              currentDir+=PID_ERROR*C_PID_P # Multiply PID error to the P constant
              currentDir+=TOTAL_ERROR*C_PID_I # Multiply the accumulated total error with I constant
              currentDir+=(PID_ERROR-PAST_ERROR) * C_PID_D # Multiple the delta between past error Nd current error with D constant
              FACE_CENTERED=False # set face centerrd to false
              #print ("LEFT",TARGET, PID_ERROR, currentDir)

      else:
          PID_ERROR = TARGET-160 # get current PID error
          if PID_ERROR>50: # optional PID error cap, to limit overshoot in movement in case error is suddenly to big
              PID_ERROR=50
          # Another check to see whether the error is within the target margin as specifidd
          if PID_ERROR<TARGET_MARGIN and PID_ERROR >-TARGET_MARGIN:
              PID_ERROR=0
              #print(TARGET,"CAPTURE_RIGHT")
              FACE_CENTERED=True
          else: # if not within the target margin, then implement the PID gains and put the value to current dir variables
              currentDir-=PID_ERROR*C_PID_P # Multiply PID error to the P constant
              currentDir-=TOTAL_ERROR*C_PID_I # Multiply the accumulated total error with I constant
              currentDir-=(PID_ERROR-PAST_ERROR) * C_PID_D # Multiply the delta between prev error and current error with D constamt
              FACE_CENTERED=False # set face.centered to false
              #print ("RIGHT",TARGET, PID_ERROR, currentDir)

      PAST_ERROR=PID_ERROR # Assign the past error variable to current PID error for the next loop
      TOTAL_ERROR+=PID_ERROR # Increment the total error variable with the current PID error for the next loops

      # SW limit check for the servo motor such that it does not go beyond its working range. as initialized abovr, the range is 0 - 1000
      if currentDir>1000:
          currentDir=1000
      elif currentDir<0:
          currentDir=0

      # ASSIGN THE CURRENTDIR VALUE TO THE MOTOR, THE MOTOR MOVES AT THIS COMMAND
      pitch.dir(currentDir)

  # if conditions that checks the timeout whether motor should go back to reset positions after certain time. all PID loop values are reset as well
  if (utime.ticks_diff(utime.ticks_ms(),COUNTER_RETURN_POS)>TIMEOUT_RETURN_POS) and BACK_POS==False:
      STATUS_BANNER="Resetting Position"
      lcd.clear()
      img=sensor.snapshot()
      drawLogo()
      drawDashboard()
      print("TIMEOUT RETURN POS")

      a = lcd.display(img)
      if currentDir>500:
          for i in range (currentDir,500,-1):
              pitch.dir(i)
              utime.sleep_ms(5)
      if currentDir<500:
          for i in range (currentDir,500,1):
              pitch.dir(i)
              utime.sleep_ms(5)

      currentDir=500
      PID_ERROR=0
      PAST_ERROR=0
      COUNTER_RETURN_POS=utime.ticks_ms()
      BACK_POS=True
      TARGET=160
      TOTAL_ERROR=0

  # check whethere continous false counter has reached 0, meaning possible stranger / intruder
  if CONTINUOUS_FALSE_COUNTER == 0:
      print ("POSSIBLE INTRUDER")
      CONTINUOUS_FALSE_COUNTER=100
      utime.sleep(1)


  N_FACE_DETECTED=0 # Reset face detected variable to 0

  # Outcome of the YoLo KPU, if code returns true that means a face(s) is detected
  if code:

      for i in code:

          COUNTER_RETURN_POS=utime.ticks_ms()
          BACK_POS=False

          #print(code)
          #print(i.objnum)

          N_FACE_DETECTED=int(i.objnum())

          if int(i.objnum())==1:

              deltaPos=0
              if PREV_FACE_LOC>i.x():
                  deltaPos=PREV_FACE_LOC-i.x()
              else:
                  deltaPos=i.x()-PREV_FACE_LOC

              if deltaPos<PREV_FACE_SIZE:
                  TARGET=i.x()+(i.w()/2)
              else:
                  TARGET=TARGET

              PREV_FACE_LOC=i.x()
              PREV_FACE_SIZE=i.w()

          a = img.draw_rectangle(i.rect())

          if FACE_CENTERED:

              STATUS_BANNER="Scanning Face"

              # Cut face and resize to 128x128

              face_cut = img.cut(i.x(), i.y(), i.w(), i.h())
              face_cut_128 = face_cut.resize(128, 128)
              a = face_cut_128.pix_to_ai()
              #a = img.draw_image(face_cut_128, (0,0))
              # Landmark for face 5 points

              fmap = kpu.forward(task_ld, face_cut_128)
              plist = fmap[:]

              le = (i.x() + int(plist[0] * i.w() - 10), i.y() + int(plist[1] * i.h()))
              re = (i.x() + int(plist[2] * i.w()), i.y() + int(plist[3] * i.h()))
              nose = (i.x() + int(plist[4] * i.w()), i.y() + int(plist[5] * i.h()))
              lm = (i.x() + int(plist[6] * i.w()), i.y() + int(plist[7] * i.h()))
              rm = (i.x() + int(plist[8] * i.w()), i.y() + int(plist[9] * i.h()))


              #a = img.draw_circle(le[0], le[1], 4)
              #a = img.draw_circle(re[0], re[1], 4)
              #a = img.draw_circle(nose[0], nose[1], 4)
              #a = img.draw_circle(lm[0], lm[1], 4)
              #a = img.draw_circle(rm[0], rm[1], 4)


              # align face to standard position
              src_point = [le, re, nose, lm, rm]
              T = image.get_affine_transform(src_point, dst_point)
              a = image.warp_affine_ai(img, img_face, T)
              a = img_face.ai_to_pix()

              img_face_small=img_face.resize(80,80)
              a = img.draw_image(img_face_small, (240,160))


              #a = img.draw_circle(int((80/i.w()*(le[0]-i.x())+240)), int((80/i.h()*le[1]-i.y())+160), 4)

              cv_re_x=int((80/i.w())*(re[0]-i.x())+240)
              cv_re_y=int((80/i.h())*(re[1]-i.y())+160)

              cv_le_x=int((80/i.w())*(le[0]-i.x())+240)+25
              cv_le_y=int((80/i.h())*(le[1]-i.y())+160)

              cv_nose_x=int((80/i.w())*(nose[0]-i.x())+240)
              cv_nose_y=int((80/i.h())*(nose[1]-i.y())+160)

              cv_rm_x=int((80/i.w())*(rm[0]-i.x())+240)
              cv_rm_y=int((80/i.h())*(rm[1]-i.y())+160)

              cv_lm_x=int((80/i.w())*(lm[0]-i.x())+240)
              cv_lm_y=int((80/i.h())*(lm[1]-i.y())+160)

              a = img.draw_circle(cv_re_x, cv_re_y, 6, color=(255,0,0))
              a = img.draw_circle(cv_le_x, cv_le_y, 6, color=(255,0,0))
              a = img.draw_cross(cv_nose_x, cv_nose_y, 6, color=(0,255,0))
              a = img.draw_circle(cv_rm_x, cv_rm_y, 6, color=(0,0,255))
              a = img.draw_circle(cv_lm_x, cv_lm_y, 6, color=(0,0,255))

              a = img.draw_line(cv_re_x, cv_re_y,cv_le_x, cv_le_y, color=(255,255,255))
              a = img.draw_line(cv_re_x, cv_re_y,cv_nose_x, cv_nose_y, color=(255,255,255))
              a = img.draw_line(cv_re_x, cv_re_y,cv_rm_x, cv_rm_y, color=(255,255,255))
              a = img.draw_line(cv_nose_x, cv_nose_y,cv_le_x, cv_le_y, color=(255,255,255))
              a = img.draw_line(cv_nose_x, cv_nose_y,cv_rm_x, cv_rm_y, color=(255,255,255))
              a = img.draw_line(cv_lm_x, cv_lm_y,cv_rm_x, cv_rm_y, color=(255,255,255))

              a = img.draw_line(cv_lm_x, cv_lm_y,cv_le_x, cv_le_y, color=(255,255,255))
              a = img.draw_line(cv_rm_x, cv_rm_y,cv_re_x, cv_re_y, color=(255,255,255))
              a = img.draw_line(cv_lm_x, cv_lm_y,cv_nose_x, cv_nose_y, color=(255,255,255))

              #print (i.w(), i.h(), cv_re_x, cv_re_y)
              #print(le[0], le[1])

              del (face_cut_128)

              # calculate face feature vector
              fmap = kpu.forward(task_fe, img_face)
              feature = kpu.face_encode(fmap[:])
              reg_flag = False
              scores = []
              for j in range(len(record_ftrs)):
                  score = kpu.face_compare(record_ftrs[j], feature)
                  scores.append(score)
              max_score = 0
              index = 0
              for k in range(len(scores)):
                  if max_score < scores[k]:
                      max_score = scores[k]
                      index = k

              # Conditions when known face is detected
              if max_score > ACCURACY:
                  #a = img.draw_string(0, 0, ("%s :%2.1f" % (
                  #    names[index], max_score)), color=(0, 255, 0), scale=2)

                  CONTINUOUS_FALSE_COUNTER=100

                  a = img.draw_rectangle(240,160,80,80,color=(0,255,0))
                  a = img.draw_rectangle(241,161,79,79,color=(0,255,0))
                  a = img.draw_rectangle(242,162,78,78,color=(0,255,0))

                  LED_GREEN.value(0)
                  LED_RED.value(1)
                  LED_BLUE.value(1)

                  if AUTH_STARTED == True:
                      #a = img.draw_string(245,220,str(AUTH_COUNTER),scale=1.5,color=(0,255,0))
                      AUTH_COUNTER += 1
                      AUTH_TIMER = utime.ticks_ms()
                      ATTENDANCE_COUNTER=utime.ticks_ms()
                      TOTAL_ERROR=0

                      for x in range (0,AUTH_COUNTER):
                          a = img.draw_rectangle(((245+(x*10)+(x*2)),222,10,15),fill=True,color=(0,255,0))

                      #print (AUTH_COUNTER)
                      if AUTH_COUNTER == AUTH_N_ACCEPTANCE+1:
                          #a = img.draw_string(0,0,"AUTHENTICATED",scale=2,color=(0,255,0))
                          a = img.draw_string(245,165,"VERIFIED",scale=1.5,color=(0,255,0))
                          #print("AUTH SUCCESS")
                          STATUS_BANNER="Face Authenticated"
                          AUTH_STARTED=False
                          AUTH_COUNTER=0
                          FACE_AUTHED=True
                          SECONDS_TICKER=utime.ticks_ms()

                          if ATTENDANCE_STATUS==False:
                              ATTENDANCE_STATUS=True
                              REMOTE_CTRL_WIFI[0]= True
                              #ret = player.play()
                              sendHTTPCommand(str(REMOTE_CTRL_WIFI[0]))
                              #player.finish()

                  if AUTH_STARTED == False:
                      AUTH_STARTED = True
                      AUTH_COUNTER += 1
                      AUTH_TIMER = utime.ticks_ms()
                      #print("Start Authenticating...")

              elif max_score>70 and max_score < ACCURACY:
                  a = img.draw_rectangle(240,160,80,80,color=(255,165,0))
                  a = img.draw_rectangle(241,161,79,79,color=(255,165,0))
                  a = img.draw_rectangle(242,162,78,78,color=(255,165,0))

                  LED_GREEN.value(1)
                  LED_RED.value(0)
                  LED_BLUE.value(1)

                  CONTINUOUS_FALSE_COUNTER-=1

              elif max_score<70:
                  a = img.draw_rectangle(240,160,80,80,color=(255,0,0))
                  a = img.draw_rectangle(241,161,79,79,color=(255,0,0))
                  a = img.draw_rectangle(242,162,78,78,color=(255,0,0))

                  LED_GREEN.value(1)
                  LED_RED.value(0)
                  LED_BLUE.value(1)

                  CONTINUOUS_FALSE_COUNTER-=1

              if AUTH_STARTED and (time.ticks_diff(time.ticks_ms(),AUTH_TIMER) > AUTH_TIME_OUT):
                  print("AUTH Timeout")
                  AUTH_STARTED=False
                  AUTH_COUNTER=0


              int_max_score=int(max_score)

              a = img.draw_string(240, 100, ("Accuracy: %2.1f" % max_score), scale=1, color=(255,255,0))
              a = img.draw_string(240, 120, ("N-Model: %2.1f" % index), scale=1, color=(126,0,255))
              a = img.draw_string(240, 140, ("FPS Count: %2.1f" % clock.fps()), scale=1, color=(0,255,255))

              a = img.draw_rectangle(240,111,80,9,color=(255,255,255))
              a = img.draw_rectangle(240,131,80,9,color=(255,255,255))
              a = img.draw_rectangle(240,151,80,9,color=(255,255,255))

              x_accuracy = int (max_score * 80 / 100)
              #img.draw_circle(x_accuracy+240,115,3,color=(255,255,0))
              a = img.draw_rectangle((241,112,x_accuracy,7),fill=True,color=(255,255,0))

              x_nmodel = int (index * 80 / TOTAL_TRAINED_FACE)
              #img.draw_circle(x_nmodel+240,135,3,color=(255,0,255))
              a = img.draw_rectangle((241,132,x_nmodel,7),fill=True,color=(126,0,255))

              x_fps = int (clock.fps() * 80 / 35)
              #img.draw_circle(x_fps+240,155,3,color=(0,255,255))
              a = img.draw_rectangle((241,152,x_fps,7),fill=True,color=(0,255,255))

              if counter_face_training > 1 and counter_face_training < 11:
                  noCal="Face Calibration No : " + str(counter_face_training)
                  a = img.draw_string(0,0,noCal, scale=2)

              if start_processing:

                  # write the current face feature to SD card
                  fileTitle="/sd/faceSumasta_"
                  fileTitle=fileTitle+str(counter_face_training)
                  fileTitle=fileTitle+".txt"
                  print (fileTitle)

                  with open(fileTitle, "w") as f:
                      f.write(feature)

                  counter_face_training = counter_face_training + 1
                  if counter_face_training > 10:
                      counter_face_training = 1

                  start_processing = False

                  #with open("/sd/faceSumasta.txt", "r") as f:
                  #    content = f.read()
                  #record_ftr = content
                  #record_ftrs.append(record_ftr)

              # Initialize sequence loop only happens once when init_processing flag is true
              # This logic is simply to load the binary files of the calibrated faces to the RAM (record ftrs)
              # This shall allow faster read and loop
              if init_processing:
                  for i in range (1,TOTAL_TRAINED_FACE):
                      fileTitleTrained="/sd/faceSumasta_"
                      fileTitleTrained=fileTitleTrained+str(i)
                      fileTitleTrained=fileTitleTrained+".txt"
                      with open(fileTitleTrained, "r") as f:
                          record_ftr = f.read()
                      #record_ftr = content
                      record_ftrs.append(record_ftr)
                  init_processing = False

                  """
                  with open("/sd/photocounter.txt", "w") as f:
                    f.write(FILE_COUNTER)

                  fileTitleTrained="/sd/photocounter.txt"
                  with open(fileTitleTrained, "r") as f:
                    FILE_COUNTER = f.read()
                  """

              break

  else: # If no face is detected then set the LED Blue to on and change the status banner, waiting for face.
      LED_GREEN.value(1)
      LED_RED.value(1)
      LED_BLUE.value(0)
      STATUS_BANNER="Waiting for Face"

  fps = clock.fps()

  a = lcd.display(img)
  #gc.collect()

  #kpu.memtest()

# a = kpu.deinit(task_fe)
# a = kpu.deinit(task_ld)
# a = kpu.deinit(task_fd)
