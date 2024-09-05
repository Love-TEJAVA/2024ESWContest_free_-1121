from flask import Flask, render_template, Response
import RPi.GPIO as GPIO
import time
import io
import threading
from picamera2 import Picamera2, Preview
from flask_socketio import SocketIO, emit
from PIL import Image

app = Flask(__name__)
socketio = SocketIO(app)

# GPIO setup for motors and servo
GPIO.setmode(GPIO.BCM)

# Motor GPIO pins
RIGHT_FORWARD = 26
RIGHT_BACKWARD = 19
LEFT_FORWARD = 16
LEFT_BACKWARD = 20
RIGHT_PWM = 13
LEFT_PWM = 21

GPIO.setup([RIGHT_FORWARD, RIGHT_BACKWARD, LEFT_FORWARD, LEFT_BACKWARD], GPIO.OUT)
GPIO.setup([RIGHT_PWM, LEFT_PWM], GPIO.OUT)

RIGHT_MOTOR = GPIO.PWM(RIGHT_PWM, 100)
LEFT_MOTOR = GPIO.PWM(LEFT_PWM, 100)
RIGHT_MOTOR.start(0)
LEFT_MOTOR.start(0)

# Ultrasonic sensor pins
TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Global variable to track forward movement
is_moving_forward = False

# Camera class for video streaming with threading to increase frame rate
class Camera:
    thread = None
    frame = None
    start_time = 0
    lock = threading.Lock()

    @classmethod
    def get_frame(cls):
        with cls.lock:
            return cls.frame

    @classmethod
    def update_frame(cls):
        picam2 = Picamera2()
        config = picam2.create_video_configuration(main={"size": (640, 480)})  # Basic camera setup
        picam2.configure(config)
        picam2.start_preview(Preview.NULL)
        picam2.start()

        stream = io.BytesIO()
        while True:
            stream.seek(0)
            picam2.capture_file(stream, format='jpeg')
            stream.seek(0)
            # Rotate the image by 180 degrees
            image = Image.open(stream)
            rotated_image = image.rotate(180)
            rotated_stream = io.BytesIO()
            rotated_image.save(rotated_stream, format='jpeg')
            rotated_stream.seek(0)
            cls.frame = rotated_stream.read()

    @classmethod
    def start_thread(cls):
        if cls.thread is None:
            cls.thread = threading.Thread(target=cls.update_frame)
            cls.thread.daemon = True
            cls.thread.start()

# Ultrasonic sensor class for distance measurement
class UltrasonicSensor:
    thread = None
    distance = 0
    lock = threading.Lock()

    @classmethod
    def get_distance(cls):
        with cls.lock:
            return cls.distance

    @classmethod
    def update_distance(cls):
        while True:
            GPIO.output(TRIG, GPIO.LOW)
            time.sleep(0.1)  # Small delay to stabilize sensor

            GPIO.output(TRIG, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(TRIG, GPIO.LOW)

            pulse_start, pulse_end = 0, 0
            while GPIO.input(ECHO) == 0:
                pulse_start = time.time()

            while GPIO.input(ECHO) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150
            distance = round(distance, 2)

            with cls.lock:
                cls.distance = distance

            # Emit the distance to the Android app
            socketio.emit('distanceUpdate', distance)

            time.sleep(1)  # Update the distance every 1 second

    @classmethod
    def start_thread(cls):
        if cls.thread is None:
            cls.thread = threading.Thread(target=cls.update_distance)
            cls.thread.daemon = True
            cls.thread.start()
            
@app.route('/distance')
def distance():
    try:
        distance = UltrasonicSensor.get_distance()
        return str(distance)
    except Exception as e:
        return "Error getting distance: " + str(e)
		
@app.route('/distance')
def distance():
    try:
        distance = UltrasonicSensor.get_distance()
        return str(distance)
    except Exception as e:
        return "Error getting distance: " + str(e)

# Function to control both motors
def move_motors(right_forward, left_forward, pwm):
    GPIO.output(RIGHT_FORWARD, right_forward)
    GPIO.output(RIGHT_BACKWARD, not right_forward)
    GPIO.output(LEFT_FORWARD, left_forward)
    GPIO.output(LEFT_BACKWARD, not left_forward)
    RIGHT_MOTOR.ChangeDutyCycle(pwm)
    LEFT_MOTOR.ChangeDutyCycle(pwm)

# Function to control car movements
def forward():
    global is_moving_forward
    is_moving_forward = True
    move_motors(True, True, 90)  # Move both motors forward
    check_distance_thread = threading.Thread(target=check_distance_while_moving)
    check_distance_thread.daemon = True
    check_distance_thread.start()

def back():
    global is_moving_forward
    is_moving_forward = False
    move_motors(False, False, 90)  # Move both motors backward
    
def turn_left():
    move_motors(True, False, 90)  # Right motor forward, left motor stop

def turn_right():
    move_motors(False, True, 90)  # Left motor forward, right motor stop

def stop_motors():
    global is_moving_forward
    is_moving_forward = False
    move_motors(False, False, 0)  # Stop both motors

def check_distance_while_moving():
    while is_moving_forward:
        distance = UltrasonicSensor.get_distance()
        if distance <= 15:
            stop_motors()  # Stop if the distance is less than or equal to 15 cm
            print(f"Obstacle detected! Stopping car. Distance: {distance} cm")
        time.sleep(0.1)  # Check every 0.1 seconds
      
# WebSocket event handling for control
@socketio.on('control')
def handle_control(data):
    command = data.get('command')

    if command == 'F':
        forward()
    elif command == 'L':
        turn_left()
    elif command == 'R':
        turn_right()
    elif command == 'B':
        back()
    elif command == 'STOP':
        stop_motors()


# Route for video streaming
@app.route('/video_feed')
def video_feed():
    Camera.start_thread()  # Ensure the camera thread is running
    def generate():
        while True:
            frame = Camera.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return render_template('video.html')

if __name__ == '__main__':
    try:
        UltrasonicSensor.start_thread()  # Start the ultrasonic sensor thread
        socketio.run(app, host='0.0.0.0', port=8300, debug=True)
    except KeyboardInterrupt:
        GPIO.cleanup()