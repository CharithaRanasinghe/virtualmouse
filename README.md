# 🤚 Advanced Hand Gesture-Based Virtual Mouse

Control your PC entirely with hand gestures using a standard webcam! This project leverages **MediaPipe**, **OpenCV**, and Python to create a responsive, touchless virtual mouse interface.

---

## 🔹 Features
- **Cursor Movement:** Move your index finger to control the cursor.
- **Left Click:** Pinch thumb + index finger.
- **Right Click:** Pinch thumb + middle finger.
- **Drag & Drop:** Pinch and hold for dragging.
- **Real-Time Visual Feedback:** See hand landmarks and gesture status.
- **Smooth Motion:** Adjustable smoothing and scaling.
- **Compatible with Physical Mouse:** Works alongside your existing mouse.

---

## 🎥 Demo
![Virtual Mouse Demo](demo.gif)  
(https://drive.google.com/file/d/1zjR9b81MNZ7mUFaNP3jT-FDDBW56vgry/view?usp=sharing)

---

## 🛠 Technology Stack
- **Python 3.x**  
- **OpenCV:** Video capture & processing  
- **MediaPipe:** Hand landmark detection  
- **ctypes:** Windows cursor control  
- **Threading:** Asynchronous mouse and video feed processing  

---

## ⚡ Installation

1. **Clone the repository:**
```bash
git clone https://github.com/yourusername/virtual-hand-mouse.git
cd virtual-hand-mouse
