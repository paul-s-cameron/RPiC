import time, threading
import RPi.GPIO as GPIO
from dataclasses import dataclass

# Set the GPIO port to BCM encoding mode
GPIO.setmode(GPIO.BCM)
# Ignore warning information
GPIO.setwarnings(False)

# Motor pin definition
@dataclass
class Motor:
    L_F: int = 20
    L_R: int = 21
    R_F: int = 19
    R_R: int = 26
    ENA: int = 16
    ENB: int = 13

# Servo pin definition
@dataclass
class Servo:
    FrontPin: int = 23      # J1
    LeftRightPin: int = 11  # J2
    UpDownPin: int = 9      # J3
    J4: int = 10            # J4
    J5: int = 25            # J5
    J6: int = 2             # J6

# Track sensors deinition
@dataclass
class TrackSensors:
    LeftPin1: int = 3
    LeftPin2: int = 5
    RightPin1: int = 4
    RightPin2: int = 18

# Ultrasonic pin definition
@dataclass
class Ultrasonic:
    EchoPin: int = 0
    TrigPin: int = 1

# Definition of RGB module pin
@dataclass
class LED:
    R: int = 22
    G: int = 27
    B: int = 24

# Misc pin definition
@dataclass
class Misc:
    Buzzer: int = 8

@dataclass
class Signal:
    pin: int
    active: int

    def __post_init__(self):
        GPIO.setup(self.pin, GPIO.IN)

pwmCache = []
@dataclass
class PWM:
    pin: int
    frequency: int
    duty_cycle: int
    default_state: int = 0
    active: bool = False
    gpio_pwm: GPIO.PWM = None

    name: str = None

    def __post_init__(self):
        GPIO.setup(self.pin, GPIO.OUT)
        self.gpio_pwm = GPIO.PWM(self.pin, self.frequency)
        pwmCache.append(self)

    def __del__(self):
        self.gpio_pwm.stop()
        # GPIO.output(self.pin, self.default_state)
        pwmCache.remove(self)

class Controller():
    def __init__(self, *, debug=False) -> None:
        '''Initialize the controller and set up the GPIO pins\n\nTakes an optional debug parameter to print debug information\n\nExample: controller = Controller(debug=True)'''
        global pwm_ENA, pwm_ENB, delaytime, gpio_code, pwm_rled, pwm_gled, pwm_bled
        global pwm_FrontServo, pwm_UpDownServo, pwm_LeftRightServo, pwm_UpDownServo_pos, pwm_LeftRightServo_pos

        self.debug = debug

        # Motor pin initialization
        GPIO.setup(Motor.ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(Motor.L_F, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor.L_R, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor.ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(Motor.R_F, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(Motor.R_R, GPIO.OUT, initial=GPIO.LOW)

        # Servo pin initialization
        GPIO.setup(Servo.FrontPin,      GPIO.OUT)
        GPIO.setup(Servo.UpDownPin,     GPIO.OUT)
        GPIO.setup(Servo.LeftRightPin,  GPIO.OUT)
        GPIO.setup(Servo.J4,            GPIO.OUT)
        GPIO.setup(Servo.J5,            GPIO.OUT)
        GPIO.setup(Servo.J6,            GPIO.OUT)

        # Track sensor pin initialization
        GPIO.setup(TrackSensors.LeftPin1,   GPIO.IN)
        GPIO.setup(TrackSensors.LeftPin2,   GPIO.IN)
        GPIO.setup(TrackSensors.RightPin1,  GPIO.IN)
        GPIO.setup(TrackSensors.RightPin2,  GPIO.IN)

        # Ultrasonic pin initialization
        GPIO.setup(Ultrasonic.EchoPin, GPIO.IN)
        GPIO.setup(Ultrasonic.TrigPin, GPIO.OUT)

        # RGB LED pin initialization
        GPIO.setup(LED.R, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LED.G, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(LED.B, GPIO.OUT, initial=GPIO.LOW)

        # Misc pin initialization
        GPIO.setup(Misc.Buzzer, GPIO.OUT, initial=GPIO.HIGH)
        
        # Set the PWM pin and frequency is 2000hz
        pwm_ENA = GPIO.PWM(Motor.ENA, 2000)
        pwm_ENB = GPIO.PWM(Motor.ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)
        
        # Set the servo frequency and starting duty cycle
        pwm_FrontServo = GPIO.PWM(Servo.FrontPin, 50)
        pwm_UpDownServo = GPIO.PWM(Servo.UpDownPin, 50)
        pwm_LeftRightServo = GPIO.PWM(Servo.LeftRightPin, 50)
        pwm_FrontServo.start(0)
        pwm_UpDownServo.start(0)
        pwm_LeftRightServo.start(0)
        pwm_rled = GPIO.PWM(LED.R, 1000)
        pwm_gled = GPIO.PWM(LED.G, 1000)
        pwm_bled = GPIO.PWM(LED.B, 1000)
        pwm_rled.start(0)
        pwm_gled.start(0)
        pwm_bled.start(0)
        
        # Recenter servos
        self.sonar_pos(90)
        self.camera_pos(75, 75)
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()

    def cleanup(self):
        '''Cleanup GPIO pins and stop all PWMs'''
        self.stop()
        for pwm in pwmCache:
            del pwm
        pwm_ENA.stop()
        pwm_ENB.stop()
        pwm_FrontServo.stop()
        pwm_UpDownServo.stop()
        pwm_LeftRightServo.stop()
        pwm_rled.stop()
        pwm_gled.stop()
        pwm_bled.stop()
        GPIO.cleanup()

    def get_signal(self, signal: Signal):
        '''Get the value of a Signal'''
        return GPIO.input(signal.pin) == signal.active
    
    #region PWM functions
    def pwm_frequency(self, pwm: PWM, *, frequency):
        '''Set the frequency of a PWM'''
        if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Setting PWM Frequency")
        if pwm.active:
            pwm.gpio_pwm.ChangeFrequency(frequency)
        pwm.frequency = frequency
        return True

    def pwm_duty(self, pwm: PWM, *, duty_cycle):
        '''Set the duty cycle of a PWM'''
        if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Setting PWM Duty Cycle")
        if pwm.active:
            pwm.gpio_pwm.ChangeDutyCycle(duty_cycle)
        pwm.duty_cycle = duty_cycle
        return True
    
    def pwm_start(self, pwm: PWM, *, duty_cycle=None):
        '''Start a PWM'''
        # Change duty cycle if active
        if duty_cycle is not None and pwm.active:
            if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Changing Duty Cycle")
            pwm.gpio_pwm.ChangeDutyCycle(duty_cycle)
            pwm.duty_cycle = duty_cycle
        # Start PWM with new duty cycle
        elif duty_cycle is not None and not pwm.active:
            if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Starting PWM with new duty cycle")
            pwm.gpio_pwm.start(duty_cycle)
            pwm.duty_cycle = duty_cycle
        # Start PWM with saved duty cycle
        elif pwm.duty_cycle is not None and not pwm.active:
            if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Starting PWM with saved duty cycle")
            pwm.gpio_pwm.start(pwm.duty_cycle)
        else:
            if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}No Duty Cycle Specified")
            # No duty cycle specified
            return False
        pwm.active = True
        return True
    
    def pwm_stop(self, pwm: PWM):
        '''Stop a PWM'''
        if self.debug: print(f"{f'[{pwm.name}] ' if pwm.name else ''}Stopping PWM")
        # Use DutyCycle to stop the PWM since it is more reliable than stop()
        pwm.gpio_pwm.ChangeDutyCycle(0 if pwm.default_state == 0 else 100)
        # # Halt the PWM
        # pwm.gpio_pwm.stop()
        # # Delete the PWM object so its state is reset
        # del pwm.gpio_pwm
        # # Give the PWM time to fully stop
        # time.sleep(1 / pwm.frequency)
        # # Set the pin to the default state
        # GPIO.output(pwm.pin, pwm.default_state)
        pwm.active = False
        return True
    #endregion
    
    #region Misc functions
    def beep(self, duration=1, frequency=1, on_time=20):
        '''Beep for a specified duration and frequency'''
        if self.debug: print(f"Beeping: {duration}, {frequency}")
        pwm_Buzzer = GPIO.PWM(Misc.Buzzer, frequency)
        pwm_Buzzer.start(100 - on_time)
        threading.Timer(duration, pwm_Buzzer.stop).start()
    #endregion

    #region LED functions
    def color(self, r: int, g: int, b: int):
        '''Set the RGB color of the LEDs 0-255'''
        if self.debug: print(f"Setting Color: {r}, {g}, {b}")
        pwm_rled.ChangeDutyCycle(r/255 * 100)
        pwm_gled.ChangeDutyCycle(g/255 * 100)
        pwm_bled.ChangeDutyCycle(b/255 * 100)

    def color(self, hex_color: str):
        '''Set the RGB color of the LEDs as a hex string'''
        if self.debug: print(f"Setting Color: {hex_color}")
        hex_color = hex_color.lstrip('#')
        lv = len(hex_color)
        rgb = tuple(int(hex_color[i:i + lv // 3], 16) for i in range(0, lv, lv // 3))
        pwm_rled.ChangeDutyCycle(rgb[0]/255 * 100)
        pwm_gled.ChangeDutyCycle(rgb[1]/255 * 100)
        pwm_bled.ChangeDutyCycle(rgb[2]/255 * 100)

    def amber(self, duration=1, frequency=1, on_time=20):
        '''Blink amber LED for a specified duration and frequency'''
        if self.debug: print(f"Blinking Amber: {duration}, {frequency}")
        pwm_Amber = GPIO.PWM(Servo.J5, frequency)
        pwm_Amber.start(on_time)
        threading.Timer(duration, pwm_Amber.stop).start()
    #endregion
        
    #region Servo movement functions
    def sonar_pos(self, pos):
        '''Set the ultrasonic position 0-180'''
        if self.debug: print(f"Setting Sonar Position: {pos}")
        pwm_FrontServo.ChangeDutyCycle(2.5 + 10 * pos/180)
        time.sleep(0.3)
        pwm_FrontServo.ChangeDutyCycle(0)
    
    def camera_pos(self, horizontal=75, vertical=75):
        '''Set the camera position pitch and yaw 0-180'''
        if self.debug: print(f"Setting Camera Position X:{horizontal} Y:{vertical}")
        pwm_UpDownServo.ChangeDutyCycle(2.5 + 10 * vertical/180)
        pwm_LeftRightServo.ChangeDutyCycle(2.5 + 10 * horizontal/180)
        time.sleep(0.3)
        pwm_LeftRightServo.ChangeDutyCycle(0)
        pwm_UpDownServo.ChangeDutyCycle(0)
    #endregion    
    
    #region Sensor functions
    def get_track(self):
        '''Get the track sensor values as a string'''
        TrackSensorLeftValue1  = GPIO.input(TrackSensors.LeftPin1)
        TrackSensorLeftValue2  = GPIO.input(TrackSensors.LeftPin2)
        TrackSensorRightValue1 = GPIO.input(TrackSensors.RightPin1)
        TrackSensorRightValue2 = GPIO.input(TrackSensors.RightPin2)
        infrared_track_value_list = ['0','0','0','0']
        infrared_track_value_list[0] = str(1 ^TrackSensorLeftValue1)
        infrared_track_value_list[1] = str(1 ^TrackSensorLeftValue2)
        infrared_track_value_list[2] = str(1 ^TrackSensorRightValue1)
        infrared_track_value_list[3] = str(1 ^TrackSensorRightValue2)
        return ''.join(infrared_track_value_list)
        
    def get_distance(self):
        '''Get the distance of the obstacle in front of the ultrasonic sensor as float'''
        if self.debug: print("Getting Distance")
        GPIO.output(Ultrasonic.TrigPin,GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(Ultrasonic.TrigPin,GPIO.LOW)
        while not GPIO.input(Ultrasonic.EchoPin):
            pass
            t1 = time.time()
        while GPIO.input(Ultrasonic.EchoPin):
            pass
            t2 = time.time()
        time.sleep(0.01)
        distance = ((t2 - t1)* 340 / 2) * 100
        if self.debug: print(f"Distance: {distance}")
        return distance
    #endregion

    #region Basic motor movement functions
    def forward(self, duration=1, speed=50):
        '''Move forward for a specified duration and speed'''
        if self.debug: print(f"Moving Forward: {duration}, {speed}")
        self.LM_forward(speed)
        self.RM_forward(speed)
        time.sleep(duration)
        self.stop()

    def reverse(self, duration=1, speed=50):
        '''Move backward for a specified duration and speed'''
        if self.debug: print(f"Moving Backward: {duration}, {speed}")
        self.LM_reverse(speed)
        self.RM_reverse(speed)
        time.sleep(duration)
        self.stop()

    def turn_left(self, duration=1, speed=50):
        '''Turn left for a specified duration and speed'''
        if self.debug: print(f"Turning Left: {duration}, {speed}")
        self.RM_forward(speed)
        time.sleep(duration)
        self.stop()

    def turn_right(self, duration=1, speed=50):
        '''Turn right for a specified duration and speed'''
        if self.debug: print(f"Turning Right: {duration}, {speed}")
        self.LM_forward(speed)
        time.sleep(duration)
        self.stop()

    def rotate_left(self, duration=1, speed=50):
        '''Rotate left for a specified duration and speed'''
        if self.debug: print(f"Rotating Left: {duration}, {speed}")
        self.LM_reverse(speed)
        self.RM_forward(speed)
        time.sleep(duration)
        self.stop()

    def rotate_right(self, duration=1, speed=50):
        '''Rotate right for a specified duration and speed'''
        if self.debug: print(f"Rotating Right: {duration}, {speed}")
        self.LM_forward(speed)
        self.RM_reverse(speed)
        time.sleep(duration)
        self.stop()

    def stop(self):
        '''Stops all track motors'''
        if self.debug: print("Stopping")
        GPIO.output(Motor.L_F, GPIO.LOW)
        GPIO.output(Motor.L_R, GPIO.LOW)
        GPIO.output(Motor.R_F, GPIO.LOW)
        GPIO.output(Motor.R_R, GPIO.LOW)
    #endregion
    
    #region Advanced motor movement functions
    def LM_forward(self, speed=50):
        '''Move left motor forward at a specified speed'''
        if self.debug: print(f"Moving Left Motor Forward: {speed}")
        pwm_ENA.ChangeDutyCycle(speed)
        GPIO.output(Motor.L_F, GPIO.HIGH)
        GPIO.output(Motor.L_R, GPIO.LOW)
        
    def LM_reverse(self, speed=50):
        '''Move left motor backward at a specified speed'''
        if self.debug: print(f"Moving Left Motor Backward: {speed}")
        pwm_ENA.ChangeDutyCycle(speed)
        GPIO.output(Motor.L_F, GPIO.LOW)
        GPIO.output(Motor.L_R, GPIO.HIGH)
        
    def RM_forward(self, speed=50):
        '''Move right motor forward at a specified speed'''
        if self.debug: print(f"Moving Right Motor Forward: {speed}")
        pwm_ENB.ChangeDutyCycle(speed)
        GPIO.output(Motor.R_F, GPIO.HIGH)
        GPIO.output(Motor.R_R, GPIO.LOW)
        
    def RM_reverse(self, speed=50):
        '''Move right motor backward at a specified speed'''
        if self.debug: print(f"Moving Right Motor Backward: {speed}")
        pwm_ENB.ChangeDutyCycle(speed)
        GPIO.output(Motor.R_F, GPIO.LOW)
        GPIO.output(Motor.R_R, GPIO.HIGH)
    #endregion