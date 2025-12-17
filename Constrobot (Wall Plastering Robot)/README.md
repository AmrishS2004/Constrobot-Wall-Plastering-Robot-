# 🤖 Constrobot  
### Vision-Powered Autonomous Wall Plastering Robot

Constrobot is an autonomous robotic system designed for **wall surface inspection and cement plastering**.  
It integrates **YOLOv8-based computer vision** running on an **NVIDIA Jetson Nano** with **embedded motor control using an Arduino Mega**.

The robot detects **wall cracks and dents**, pauses vision processing, performs a **plastering operation at the detected location**, and then resumes inspection automatically.

---

## 📸 Prototype

![Constrobot Prototype](https://github.com/AmrishS2004/Constrobot-Wall-Plastering-Robot-/blob/main/Constrobot%20(Wall%20Plastering%20Robot)/Constrobot_Prototype.JPG)

*Figure: Constrobot prototype with vertical motion mechanism and plastering module*

---

## 📁 Repository Structure

```
Constrobot/
│
├── Arduino/
│   └── Arduino_Mega_code.ino
│
├── Jetson/
│   └── Jetson_Master.py
│
├── Model/
│   └── best.pt
│
├── images/
│   └── constrobot_prototype.jpg
│
└── README.md
```

---

## 🧠 System Architecture

```
Camera → Jetson Nano (YOLOv8 Detection)
               │
               │ UART Communication
               ▼
          Arduino Mega
               │
     Motors / Servos / Plastering Unit
```

---

## ⚙️ Hardware Components

- NVIDIA Jetson Nano (4GB)
- Arduino Mega 2560
- USB / CSI Camera
- Stepper motor with lead screw (vertical motion)
- DC motor (plastering mechanism)
- BTS7960 motor driver
- Servo motors
- Aluminium frame structure
- External motor power supply

> ⚠️ Ensure **common ground** between Jetson Nano and Arduino Mega.

---

## 💻 Software Requirements

### Jetson Nano
- Ubuntu (JetPack)
- Python 3.8+
- PyTorch (Jetson compatible)
- OpenCV
- Ultralytics YOLOv8
- pySerial

### Arduino
- Arduino IDE
- Servo library
- Stepper / motor driver libraries

---

## 🔌 UART Communication Setup

| Jetson Nano | Arduino Mega |
|------------|--------------|
| TX (GPIO14) | RX (Pin 19) |
| RX (GPIO15) | TX (Pin 18) |
| GND | GND |

Disable serial console on Jetson:
```bash
sudo systemctl disable nvgetty
```

---

## 🧠 YOLOv8 Model Configuration

1. Place the trained model in:
```
Model/best.pt
```

2. Load the model in `Jetson_Master.py`:
```python
from ultralytics import YOLO
model = YOLO("Model/best.pt")
```

3. Ensure class labels match training:
```python
classes = ["crack", "dent"]
```

---

## 🚀 Running the System

### Step 1: Arduino Setup
1. Open `Arduino_Mega_code.ino`
2. Select **Board: Arduino Mega 2560**
3. Select correct serial port
4. Upload the code

---

### Step 2: Jetson Nano Execution
```bash
python3 Jetson_Master.py
```

---

## 🔄 Operational Workflow

1. Jetson Nano continuously processes camera frames using YOLOv8
2. On detection of a crack or dent:
   - Jetson sends a UART signal to Arduino
3. Arduino:
   - Stores the defect location
   - Pauses vision processing
   - Executes plastering sequence
   - Stops all motors after completion
4. Arduino signals Jetson to resume detection
5. Robot continues inspection

---

## 📡 UART Command Protocol (Example)

| Command | Description |
|--------|------------|
| `D` | Defect detected |
| `P` | Pause detection |
| `R` | Resume detection |
| `S` | Stop motors |

---

## 🛠️ Configuration & Tuning

- Motor angles, delays, and distances:
  ```
  Arduino/Arduino_Mega_code.ino
  ```
- Detection confidence threshold:
  ```python
  model(frame, conf=0.5)
  ```
- Camera index:
  ```python
  cv2.VideoCapture(0)
  ```

---

## 🧪 Testing Checklist

- UART communication verified
- Motors calibrated
- YOLOv8 model loads correctly
- Pause/resume logic working
- Full plastering cycle completed

---

## 📌 Project Status

- ✔️ Vision system implemented
- ✔️ Embedded control integrated
- ✔️ Patent filed
- 🚧 Testing and calibration ongoing

---

## 📜 License

This project is intended for **academic and research purposes only**.  
Commercial or industrial usage requires prior permission.

---

## 👤 Author

**Amrish Sasikumar**  
Robotics | Computer Vision | Embedded Systems  
Incoming MS in Computer Science, Arizona State University
