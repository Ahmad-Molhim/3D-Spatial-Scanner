An embedded system that performs 360° spatial scanning using a VL53L1X time-of-flight (ToF) distance sensor mounted on a stepper motor. Controlled by an MSP432 microcontroller, the system transmits data to a PC via UART, where it is visualized in 3D using Python and Open3D.

---

## 🎯 Features

- ⏱️ Accurate distance measurement using VL53L1X ToF sensor (via I2C)
- ⚙️ Stepper motor control using ULN2003 driver for 360° rotation
- 🔄 Real-time data acquisition via MSP432 (ARM Cortex-M4)
- 🧠 UART serial communication to PC at 115200 baud
- 📊 3D visualization using Python (PySerial + NumPy + Open3D)
- 🖨️ 3D-printed sensor and motor mounts for modular hardware

---

## 🛠️ Technologies Used

- **MSP-EXP432E401Y** (Keil uVision IDE)
- **C** for embedded firmware
- **Python 3.9** with:
  - `pyserial`
  - `numpy`
  - `open3d`
- **I2C** and **UART** communication protocols
- **KiCad** (for schematic)
- **3D printing** (for mounts and hardware integration)

---

## 🖥️ How It Works

1. The ToF sensor takes distance measurements every 11.25° while the stepper motor completes 3 full rotations.
2. Measurements are sent from the microcontroller to a PC over UART.
3. Python code converts the distance values into 3D `x, y, z` coordinates.
4. The scan is visualized using Open3D in real-time.
