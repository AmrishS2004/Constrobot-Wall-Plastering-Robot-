import serial
import time
from ultralytics import YOLO
import cv2

# Load your YOLO model (change as needed)
model = YOLO('your_model.pt')  # Replace with actual model path

# Serial port (adjust if needed)
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

# Flags
detecting = False
mark_sent = False
started = False

# Camera (update if using CSI or USB cam)
cap = cv2.VideoCapture(0)

def send_to_arduino(msg):
    arduino.write((msg + '\n').encode())
    print(f"> Sent to Arduino: {msg}")

def listen_to_arduino():
    global detecting, mark_sent
    if arduino.in_waiting:
        line = arduino.readline().decode().strip()
        print(f"< From Arduino: {line}")
       
        if line == "DOWN_START":
            detecting = True
            mark_sent = False
        elif line == "RESUME_DETECT":
            detecting = True
            mark_sent = False
        elif line == "DETECT_STOP":
            detecting = False
            mark_sent = False

# Step 1: Send START to Arduino
if not started:
    send_to_arduino("START")
    started = True

print("Jetson: Waiting for Arduino to reach DOWN_START...")

while True:
    listen_to_arduino()

    if detecting:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model.predict(frame, imgsz=640, conf=0.4)
        for result in results:
            names = result.names
            for box in result.boxes:
                cls = int(box.cls[0])
                label = names[cls]
               
                if label in ['hallow', 'seam'] and not mark_sent:
                    print(f"Detected {label} — sending MARK")
                    send_to_arduino("MARK")
                    mark_sent = True
                    detecting = False  # Pause detection until Arduino responds

        # Optional: Show the camera feed
        # cv2.imshow("YOLO Detection", frame)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

    time.sleep(0.1)

cap.release()
arduino.close()

