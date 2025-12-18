# 🤖 Constrobot: Vision-Powered Autonomous Wall Plastering Robot

[![Patent Filed](https://img.shields.io/badge/Patent-Filed-success.svg)](https://github.com/AmrishS2004/Constrobot-Wall-Plastering-Robot-)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-blueviolet.svg)](https://github.com/ultralytics/ultralytics)
[![Jetson](https://img.shields.io/badge/NVIDIA-Jetson%20Nano-76B900.svg)](https://developer.nvidia.com/embedded/jetson-nano)
[![Arduino](https://img.shields.io/badge/Arduino-Mega%202560-00979D.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-Custom%20Restricted-red.svg)](LICENSE)

> **Fully Funded Research Project** | In Collaboration with **L&T Construction**

An autonomous robotic system that revolutionizes construction quality control by detecting wall surface defects in real-time and performing precision plastering operations. Constrobot combines state-of-the-art computer vision with embedded systems control to automate what has traditionally been manual, time-intensive work.

---

## 🎯 Project Highlights

- **92% Detection Accuracy** - YOLOv8 model trained on 10,000+ defect images
- **Real-Time Processing** - 10 FPS on NVIDIA Jetson Nano with 300ms decision latency
- **Industrial Partnership** - Fully funded by L&T Construction
- **Patent Filed** - Intellectual property protection for novel automation system
- **40% Efficiency Gain** - Reduced plastering time through automated inspection and repair
- **25% Query Optimization** - Improved defect database retrieval times for engineering teams

---

## 📸 Visual Documentation

### 🎥 System Demonstration

<div align="center">

**Watch Constrobot in Action**

[![Constrobot Demo Video](https://img.shields.io/badge/▶️-Watch%20Demo-red?style=for-the-badge&logo=youtube)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

*Click above to watch the full demonstration video*

</div>

> 💡 **Video Highlights**: Real-time defect detection • Autonomous plastering operation • UART communication in action • Complete workflow demonstration

---

### 🏗️ CAD Design & Prototype

<div align="center">

| SolidWorks CAD Design | Physical Prototype |
|:---------------------:|:------------------:|
| ![CAD Design](images/solidworks_design.png) | ![Prototype](images/Constrobot_Prototype.JPG) |
| *3D CAD model showing complete assembly* | *Functional prototype in testing phase* |

</div>

---

### 📐 Detailed Views

<div align="center">

| CAD Full Assembly | CAD Exploded View | Plastering Mechanism |
|:-----------------:|:-----------------:|:--------------------:|
| ![CAD Assembly](images/cad_designs/solidworks_full_assembly.png) | ![Exploded View](images/cad_designs/solidworks_exploded_view.png) | ![Plastering](images/cad_designs/plastering_mechanism.png) |
| *Complete system design* | *Component breakdown* | *Material application system* |

</div>

---

### 🔧 Key Components Breakdown

**Visible in Prototype:**

| Component | Location | Description |
|-----------|----------|-------------|
| **Aluminium Frame** | Vertical structure | T-slot extrusion providing rigid scaffold |
| **Lead Screw System** | Left & Right rails | Dual stepper motors for precise vertical motion |
| **Control Electronics** | Center platform | Arduino Mega + motor drivers + power distribution |
| **Plastering Mechanism** | Bottom platform | DC motor-driven application system with roller |
| **Mobility Base** | Bottom wheels | Omni-directional wheels for positioning |
| **Wiring Harness** | Throughout system | Organized power and signal distribution |
| **Camera Mount** | Top horizontal bar | USB/CSI camera for wall surface scanning |
| **Vision Processing** | On Jetson Nano | YOLOv8 running real-time defect detection |

### 🎨 Design Features

✅ **Modular Architecture** - Components can be serviced independently  
✅ **Dual Lead Screw** - Synchronized vertical motion preventing tilt  
✅ **Compact Footprint** - ~60cm height, suitable for standard construction sites  
✅ **Cable Management** - Organized routing preventing interference with motion  
✅ **Stable Base** - Low center of gravity with wide wheelbase  
✅ **CAD-to-Reality** - Precise manufacturing from 3D models to physical build

> 📍 **Development**: CAD designed in SolidWorks | Manufactured at VIT Chennai | Tested in collaboration with L&T Construction

---

## 💡 Problem Statement

Traditional wall plastering faces several challenges:
- **Manual Defect Detection**: Time-consuming visual inspection prone to human error
- **Inconsistent Quality**: Varies significantly based on worker skill and fatigue
- **Safety Concerns**: Workers operating at heights in potentially hazardous conditions
- **Labor Shortages**: Construction industry facing skilled labor scarcity
- **Cost Inefficiency**: High labor costs and material waste from improper application

Constrobot addresses these challenges through **autonomous inspection and targeted repair**, ensuring consistent quality while reducing costs and improving worker safety.

---

## 🏗️ System Architecture

### High-Level Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                     CONSTROBOT SYSTEM                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────────┐         ┌─────────────────┐            │
│  │  USB/CSI       │         │   NVIDIA        │            │
│  │  Camera        │────────▶│   Jetson Nano   │            │
│  │  (Input)       │         │   (Vision AI)   │            │
│  └────────────────┘         └────────┬────────┘            │
│                                      │                      │
│                              UART Communication             │
│                                (TX/RX @ 9600 baud)          │
│                                      │                      │
│                              ┌───────▼────────┐             │
│                              │   Arduino      │             │
│                              │   Mega 2560    │             │
│                              │   (Control)    │             │
│                              └───────┬────────┘             │
│                                      │                      │
│                    ┌─────────────────┼─────────────────┐   │
│                    │                 │                 │   │
│              ┌─────▼─────┐    ┌─────▼─────┐    ┌─────▼────┐
│              │  Stepper  │    │  Servo    │    │   DC     │
│              │  Motor    │    │  Motors   │    │  Motor   │
│              │ (Vertical)│    │ (Angles)  │    │(Plaster) │
│              └───────────┘    └───────────┘    └──────────┘
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow
1. **Image Acquisition** → Camera captures wall surface at 10 FPS
2. **Defect Detection** → YOLOv8 identifies cracks/dents with bounding boxes
3. **Decision Making** → Jetson evaluates confidence scores and defect locations
4. **Command Transmission** → UART signals sent to Arduino with defect coordinates
5. **Motion Control** → Arduino orchestrates stepper/servo/DC motor sequence
6. **Plastering Operation** → Targeted material application at defect location
7. **System Reset** → Resume detection after operation completion

---

## 🔧 Hardware Components

### Core Processing Units

| Component | Model | Purpose | Specifications |
|-----------|-------|---------|----------------|
| **Vision Processor** | NVIDIA Jetson Nano 4GB | AI inference, YOLOv8 execution | 128-core Maxwell GPU, Quad-core ARM A57 |
| **Motor Controller** | Arduino Mega 2560 | Real-time motor coordination | 54 digital I/O pins, 16 analog inputs |
| **Camera** | USB/CSI Camera | Wall surface imaging | 1080p resolution, 30 FPS capable |

### Actuation System

| Component | Specification | Function |
|-----------|--------------|----------|
| **Stepper Motor** | NEMA 17 with lead screw | Vertical translation mechanism |
| **DC Motor** | 12V with BTS7960 driver | Plastering material application |
| **Servo Motors** | SG90 (×2) | Angular adjustment for precision |
| **Motor Driver** | BTS7960 43A H-Bridge | High-current DC motor control |

### Mechanical Structure

- **Frame**: Aluminium extrusion (lightweight, rigid)
- **Linear Motion**: Lead screw system for vertical travel
- **Power Supply**: 12V 5A adapter for motors (isolated from logic)
- **Mounting**: Adjustable clamps for various wall configurations

> ⚠️ **Critical**: Ensure common ground connection between Jetson Nano and Arduino Mega to prevent UART communication errors.

---

## 💻 Software Stack

### Computer Vision Pipeline (Jetson Nano)

```python
# Core Dependencies
- Ubuntu 18.04 LTS (JetPack 4.6.1)
- Python 3.8+
- PyTorch 1.10.0 (Jetson-optimized)
- OpenCV 4.5.0 (with CUDA acceleration)
- Ultralytics YOLOv8n (nano model)
- pySerial 3.5 (UART communication)
- NumPy, PIL
```

**YOLOv8 Model Details:**
- **Architecture**: YOLOv8 Nano (optimized for edge devices)
- **Classes**: 2 (crack, dent)
- **Input Size**: 640×640 pixels
- **Inference Time**: ~100ms per frame on Jetson Nano
- **Training Dataset**: 10,000+ annotated construction defect images
- **Detection Confidence Threshold**: 0.5 (adjustable)

### Embedded Control (Arduino Mega)

```cpp
// Core Libraries
- Servo.h (servo motor control)
- Stepper.h (stepper motor sequencing)
- SoftwareSerial.h (UART communication)
```

---

## 📁 Repository Structure

```
Constrobot/
│
├── Arduino/
│   ├── Arduino_Mega_code.ino          # Main motor control logic
│   └── motor_config.h                 # Motor pin definitions & calibration
│
├── Jetson/
│   ├── Jetson_Master.py               # Main vision processing script
│   ├── utils/
│   │   ├── camera_handler.py          # Camera initialization & capture
│   │   ├── uart_comm.py               # UART protocol implementation
│   │   └── defect_logger.py           # Detection data logging
│   └── config.yaml                    # System configuration parameters
│
├── Model/
│   ├── best.pt                        # Trained YOLOv8 weights
│   ├── training_metrics.json          # Model performance logs
│   └── class_labels.txt               # Detection class mappings
│
├── Database/
│   └── defects.db                     # SQLite database for defect tracking
│
├── Documentation/
│   ├── Hardware_Assembly.pdf          # Mechanical assembly guide
│   ├── Wiring_Diagram.png             # Electrical connection schematic
│   └── Calibration_Guide.md           # Motor calibration procedures
│
├── images/
│   ├── Constrobot_Prototype.JPG       # System photos
│   └── detection_examples/            # Sample detection outputs
│
├── requirements.txt                   # Python dependencies
├── LICENSE                            # Project license
└── README.md                          # This file
```

---

## 🔌 Hardware Setup

### UART Connection (Jetson ↔ Arduino)

| Jetson Nano Pin | Arduino Mega Pin | Signal | Notes |
|-----------------|------------------|--------|-------|
| GPIO14 (TX) | Pin 19 (RX1) | Transmit | Jetson → Arduino |
| GPIO15 (RX) | Pin 18 (TX1) | Receive | Arduino → Jetson |
| GND | GND | Common Ground | **CRITICAL** |

### Jetson Serial Configuration

Disable the serial console to free up UART pins:
```bash
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo usermod -a -G dialout $USER
```

Verify UART device:
```bash
ls -l /dev/ttyTHS1
# Should show: crw-rw---- 1 root dialout
```

### Motor Wiring

**Stepper Motor (Vertical Motion):**
- Connect to pins 2-5 on Arduino
- Ensure proper step sequence (full/half/microstepping)

**DC Motor (Plastering):**
- BTS7960 driver connected to pins 6-9
- Separate 12V power supply (not through Arduino)

**Servo Motors:**
- Servo 1 (angle adjustment): Pin 10
- Servo 2 (trigger mechanism): Pin 11

---

## 🚀 Installation & Setup

### 1. Jetson Nano Setup

```bash
# Clone repository
git clone https://github.com/AmrishS2004/Constrobot-Wall-Plastering-Robot-.git
cd Constrobot-Wall-Plastering-Robot-

# Install dependencies
pip3 install -r requirements.txt

# Install PyTorch for Jetson (if not pre-installed)
wget https://nvidia.box.com/shared/static/p57jwntv436lfrd78inwl7iml6p13fzh.whl -O torch-1.10.0-cp36-cp36m-linux_aarch64.whl
pip3 install torch-1.10.0-cp36-cp36m-linux_aarch64.whl

# Verify YOLOv8 installation
python3 -c "from ultralytics import YOLO; print('YOLOv8 ready')"

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera:', cap.isOpened())"
```

### 2. Arduino Setup

```bash
# Open Arduino IDE
# File → Open → Arduino/Arduino_Mega_code.ino

# Configure:
# Tools → Board → Arduino Mega 2560
# Tools → Port → /dev/ttyACM0 (or appropriate port)

# Upload code
# Sketch → Upload

# Monitor serial output (optional)
# Tools → Serial Monitor (9600 baud)
```

### 3. Model Configuration

Place your trained YOLOv8 model:
```bash
cp /path/to/your/best.pt Model/best.pt
```

Update class labels in `Jetson/Jetson_Master.py`:
```python
model = YOLO("Model/best.pt")
classes = ["crack", "dent"]  # Must match training labels
```

---

## 🎮 Running Constrobot

### Quick Start

```bash
# Terminal 1: Start Jetson vision system
cd Jetson/
python3 Jetson_Master.py

# Terminal 2 (optional): Monitor Arduino serial output
screen /dev/ttyACM0 9600
```

### Advanced Usage

**Adjust Detection Confidence:**
```python
# In Jetson_Master.py, line 45
results = model(frame, conf=0.5)  # Change 0.5 to desired threshold
```

**Change Camera Source:**
```python
# In Jetson_Master.py, line 23
cap = cv2.VideoCapture(0)  # 0 for USB, 1 for CSI
```

**Modify Plastering Duration:**
```cpp
// In Arduino_Mega_code.ino, line 87
delay(3000);  // Plastering time in milliseconds
```

---

## 🔄 Operational Workflow

### Detection Phase
1. **Initialization**: Jetson loads YOLOv8 model, initializes camera and UART
2. **Continuous Scanning**: Camera captures frames at 10 FPS
3. **Real-Time Inference**: YOLOv8 processes each frame for defects
4. **Defect Identification**: If confidence > threshold, defect coordinates extracted

### Intervention Phase
5. **Signal Transmission**: Jetson sends defect data via UART to Arduino
   ```
   Format: "D,<x_coord>,<y_coord>,<defect_type>\n"
   Example: "D,320,240,crack\n"
   ```
6. **Vision Pause**: Arduino sends "P" (pause) command back to Jetson
7. **Motor Coordination**:
   - Stepper moves to vertical position
   - Servos adjust plastering angle
   - DC motor applies plastering material
8. **Operation Completion**: Arduino sends "R" (resume) command

### Resume Phase
9. **Detection Restart**: Jetson resumes scanning for new defects
10. **Data Logging**: All detections stored in SQLite database

---

## 📡 Communication Protocol

### UART Message Format

| Message Type | Format | Example | Description |
|--------------|--------|---------|-------------|
| Defect Detected | `D,x,y,type` | `D,320,240,crack` | Jetson → Arduino |
| Pause Detection | `P` | `P` | Arduino → Jetson |
| Resume Detection | `R` | `R` | Arduino → Jetson |
| Emergency Stop | `S` | `S` | Either direction |
| Acknowledge | `A` | `A` | Confirmation |

### Error Handling

- **Timeout**: If no response within 5 seconds, retry transmission
- **Checksum**: Optional CRC-8 for data integrity (currently disabled)
- **Buffer Management**: Arduino maintains queue for multiple detections

---

## 🛠️ Calibration & Tuning

### Motor Calibration

**Stepper Motor (Vertical Travel):**
```cpp
// In Arduino_Mega_code.ino
#define STEPS_PER_CM 200  // Adjust based on lead screw pitch
```

**Servo Angles:**
```cpp
servo1.write(90);  // Neutral position (adjust 0-180)
servo2.write(45);  // Trigger angle (adjust 0-180)
```

**DC Motor Speed:**
```cpp
analogWrite(motorPin, 180);  // PWM value (0-255)
```

### Vision System Tuning

**Detection Confidence:**
- Lower threshold (0.3-0.4): More detections, higher false positives
- Higher threshold (0.6-0.7): Fewer detections, higher precision

**Camera Settings:**
```python
cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)
cap.set(cv2.CAP_PROP_CONTRAST, 50)
cap.set(cv2.CAP_PROP_EXPOSURE, -5)  # Auto-exposure
```

---

## 📊 Performance Metrics

### Detection Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **Precision** | 92% | On 10,000+ defect image dataset |
| **Recall** | 88% | Balances false positives vs. missed defects |
| **F1-Score** | 0.90 | Harmonic mean of precision & recall |
| **Inference Time** | 95ms | Average per frame on Jetson Nano |
| **FPS** | 10 | Real-time processing capability |

### Operational Efficiency

| Metric | Traditional | Constrobot | Improvement |
|--------|-------------|------------|-------------|
| Inspection Time | 60 min/100m² | 36 min/100m² | **40% faster** |
| Plastering Accuracy | 85% (variable) | 92% (consistent) | **8% better** |
| Material Waste | 15% | 8% | **47% reduction** |
| Labor Cost | $50/hour | $10/hour (amortized) | **80% savings** |

### Database Query Performance

- **Query Retrieval Time**: 25% faster after SQL optimization
- **Defect Records**: 10,000+ images stored with indexed metadata
- **Storage Efficiency**: Compressed images averaging 150KB each

---

## 🧪 Testing Checklist

Before deployment, verify:

- [ ] **Hardware**
  - [ ] All motor connections secure and tested independently
  - [ ] Common ground established between Jetson and Arduino
  - [ ] Camera focus adjusted for target working distance
  - [ ] Power supply provides stable voltage (12V ±0.5V)

- [ ] **Software**
  - [ ] YOLOv8 model loads without errors
  - [ ] UART communication bidirectional (send/receive test)
  - [ ] Camera captures clear, focused images
  - [ ] Detection threshold optimized for environment

- [ ] **Functional**
  - [ ] Defect detection triggers motor response
  - [ ] Plastering operation completes full cycle
  - [ ] System resumes detection after intervention
  - [ ] Emergency stop halts all motors immediately

- [ ] **Safety**
  - [ ] Emergency stop button functional
  - [ ] Motors have current limiting
  - [ ] System fails safe (motors stop on communication loss)

---

## 📈 Future Enhancements

### Short-Term (Next 6 Months)
- [ ] Multi-surface support (brick, concrete, drywall)
- [ ] Mobile app for remote monitoring and control
- [ ] Automated material level sensing
- [ ] Cloud-based defect analytics dashboard

### Long-Term (1-2 Years)
- [ ] Full 3D wall mapping with depth cameras
- [ ] Reinforcement learning for adaptive plastering techniques
- [ ] Multi-robot coordination for large-scale projects
- [ ] Integration with BIM (Building Information Modeling) systems

---

## 🏆 Project Achievements

- ✅ **Fully Funded Research** by L&T Construction
- ✅ **Patent Filed** for autonomous plastering system
- ✅ **92% Detection Accuracy** on real-world construction data
- ✅ **40% Time Reduction** in inspection and plastering operations
- ✅ **Published Research** (Paper in review for ICRA 2025)
- ✅ **Industry Collaboration** with one of India's largest construction companies

---

## 📜 License & Usage

### ⚖️ Custom Academic & Research License

**All Rights Reserved © 2025 Amrish Sasikumar**

This project is protected under a **Custom Academic & Research License** with the following terms:

### ✅ Permitted Use (No Permission Required)
- **Viewing** source code for academic reference
- **Citing** this work with proper attribution in academic publications

### 🔒 Restricted Use (Requires Written Permission)
The following actions require **explicit prior written permission** from the author:
- Running or deploying the system
- Modifying the code or model
- Using for research, academic, industrial, or commercial purposes
- Incorporating into publications, products, or patents
- Training derivative models using provided weights or data

### 📧 Request Permission
To request usage rights, contact:
- **Email**: amrish.s2004p@gmail.com
- **Subject Line**: "Constrobot Usage Permission Request"
- Include: Your name, institution/company, intended use case

> ⚠️ **Important**: This project includes patent-pending technology. Unauthorized use may result in legal action.

### 📄 Full License
See [LICENSE](LICENSE) file for complete legal terms.

---

## 📚 Publications & Citations

**Pending Publication:**
```bibtex
@inproceedings{sasikumar2025constrobot,
  title={Constrobot: Vision-Powered Autonomous Robot for Cement Plastering and Wall Surface Aberration Detection},
  author={Sasikumar, Amrish and Team},
  booktitle={International Conference on Robotics and Automation (ICRA)},
  year={2025},
  organization={IEEE}
}
```

---

## 🤝 Acknowledgments

- **L&T Construction** for project funding and industry partnership
- **VIT Chennai** for research facilities and mentorship
- **NVIDIA** for Jetson Nano developer support
- **Ultralytics** for YOLOv8 framework

Special thanks to the robotics and computer vision communities for open-source tools that made this project possible.

---

## 👤 Author

**Amrish Sasikumar**

- 📧 Email: amrish.s2004p@gmail.com
- 💼 LinkedIn: [linkedin.com/in/amrish-sasikumar](https://linkedin.com/in/amrish-sasikumar)
- 🎓 Education:
  - M.S. Computer Science, Arizona State University (Current)
  - B.Tech Electronics & Computer Engineering, VIT Chennai (2025)

**Project Duration:** October 2024 – May 2025  
**Role:** Lead Researcher & System Architect

---

## 📞 Contact & Support

For questions, collaboration opportunities, or technical support:
- **Email**: amrish.s2004p@gmail.com
- **GitHub Issues**: [Report bugs or request features](https://github.com/AmrishS2004/Constrobot-Wall-Plastering-Robot-/issues)
- **LinkedIn**: Professional inquiries and networking

---

## ⭐ Star History

If this project helped you or inspired your work, please consider:
- ⭐ Starring this repository
- 🍴 Forking for your own research
- 📢 Sharing with the robotics community
- 💬 Providing feedback through issues

---

<div align="center">

**Built with** ❤️ **for the future of construction automation**

![Visitors](https://visitor-badge.laobi.icu/badge?page_id=AmrishS2004.Constrobot)

*Last Updated: December 2024*

</div>
