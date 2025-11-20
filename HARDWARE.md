# Hardware Setup Guide

## Required Hardware Components

### Core Components
1. **Raspberry Pi 5** (4GB or 8GB RAM recommended)
2. **USB Webcam** or **Raspberry Pi Camera Module 3**
3. **Balance Car Chassis** with two DC motors
4. **Motor Driver Board** (Choose one):
   - L298N Dual H-Bridge
   - TB6612FNG Motor Driver
   - DRV8833 Motor Driver
5. **Power Supply**:
   - 5V/3A for Raspberry Pi 5
   - 6-12V for motors (depends on motor specifications)
6. **Battery Pack** (for mobile operation)
   - LiPo battery (2S-3S) or
   - 18650 battery holder (2-3 cells)

### Optional but Recommended
7. **IMU Sensor** (for better balance):
   - MPU6050 (I2C interface)
   - BNO055 (I2C interface)
   - ICM-20948 (I2C interface)
8. **Buck Converter** (to power Pi from battery)
9. **Wheel Encoders** (for odometry)

## Wiring Diagram

### Basic Motor Control Setup (L298N Example)

```
Raspberry Pi 5                    L298N Motor Driver
┌─────────────┐                  ┌──────────────────┐
│             │                  │                  │
│   GPIO 12 ──┼──────────────────┤ ENA (PWM)        │
│   GPIO 16 ──┼──────────────────┤ IN1              │
│   GPIO 20 ──┼──────────────────┤ IN2              │
│             │                  │                  │
│   GPIO 13 ──┼──────────────────┤ ENB (PWM)        │
│   GPIO 19 ──┼──────────────────┤ IN3              │
│   GPIO 26 ──┼──────────────────┤ IN4              │
│             │                  │                  │
│   GND    ───┼──────────────────┤ GND              │
│             │                  │                  │
└─────────────┘                  │  OUT1 ───────────┤───┐
                                 │  OUT2 ───────────┤───┼─ Left Motor
                                 │  OUT3 ───────────┤───┐
                                 │  OUT4 ───────────┤───┼─ Right Motor
                                 │                  │
                                 │  +12V ←──────────┤─── Battery (+)
                                 │  GND  ←──────────┤─── Battery (-)
                                 └──────────────────┘
```

### Pin Assignments (Raspberry Pi 5)

| Function | GPIO Pin | Physical Pin | Notes |
|----------|----------|--------------|-------|
| Left Motor PWM | GPIO 12 | Pin 32 | Hardware PWM0 |
| Left Motor DIR1 | GPIO 16 | Pin 36 | Direction control |
| Left Motor DIR2 | GPIO 20 | Pin 38 | Direction control |
| Right Motor PWM | GPIO 13 | Pin 33 | Hardware PWM1 |
| Right Motor DIR1 | GPIO 19 | Pin 35 | Direction control |
| Right Motor DIR2 | GPIO 26 | Pin 37 | Direction control |
| GND | GND | Pin 6, 9, 14, 20, 25, 30, 34, 39 | Common ground |

### IMU Connection (MPU6050 via I2C)

```
Raspberry Pi 5                    MPU6050
┌─────────────┐                  ┌──────────┐
│   3.3V   ───┼──────────────────┤ VCC      │
│   GPIO 2 ───┼──────────────────┤ SDA      │
│   GPIO 3 ───┼──────────────────┤ SCL      │
│   GND    ───┼──────────────────┤ GND      │
└─────────────┘                  └──────────┘
```

### Camera Connection

**Option 1: USB Webcam**
- Simply plug into any USB port on Raspberry Pi 5
- Will appear as `/dev/video0` (or higher number)

**Option 2: Raspberry Pi Camera Module 3**
- Connect to the CSI camera connector on Raspberry Pi 5
- Enable camera in `/boot/firmware/config.txt`
- Will appear as `/dev/video0`

## Power Supply Recommendations

### Mobile Setup (Battery Powered)
```
Battery Pack (11.1V LiPo)
    │
    ├──→ Buck Converter (5V/3A) ──→ Raspberry Pi 5
    │
    └──→ Motor Driver (direct) ──→ Motors
```

### Desktop Testing Setup
```
USB-C Power Supply (5V/3A) ──→ Raspberry Pi 5

Separate DC Power Supply (6-12V) ──→ Motor Driver ──→ Motors
```

**⚠️ Important:** 
- Always use separate power for motors and Raspberry Pi
- Connect all GND together (common ground)
- Do NOT power motors from Raspberry Pi's 5V rail

## Assembly Instructions

### Step 1: Mount Components on Chassis
1. Secure Raspberry Pi 5 to the top deck of chassis
2. Mount motor driver near the motors
3. Attach battery holder underneath
4. Mount webcam at front, facing forward
5. If using IMU, mount it at the center of chassis

### Step 2: Motor Wiring
1. Connect motors to motor driver outputs
2. Test motor direction (may need to swap wires)
3. Secure all wiring with zip ties

### Step 3: Power Connections
1. Connect battery to motor driver power input
2. Connect buck converter input to battery
3. Connect buck converter output to Raspberry Pi
4. Add power switch for easy on/off

### Step 4: Signal Connections
1. Connect GPIO pins as per diagram above
2. Use dupont wires or soldered connections
3. Double-check all connections before powering on

### Step 5: IMU Connection (Optional)
1. Connect IMU to I2C pins (GPIO 2, 3)
2. Enable I2C in `raspi-config`
3. Test with `i2cdetect -y 1`

## Hardware Testing

### Test 1: Motor Driver
```bash
# Install lgpio
pip3 install lgpio

# Test motor control
python3 << EOF
import lgpio
import time

h = lgpio.gpiochip_open(0)

# Set up pins as outputs
LEFT_PWM = 12
lgpio.gpio_claim_output(h, LEFT_PWM)

# Test PWM
for i in range(0, 100, 10):
    lgpio.tx_pwm(h, LEFT_PWM, 1000, i)
    time.sleep(0.5)

lgpio.gpiochip_close(h)
EOF
```

### Test 2: Camera
```bash
# Check camera device
ls -l /dev/video*

# Capture test image
fswebcam test.jpg

# Or use OpenCV
python3 -c "import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); cv2.imwrite('test.jpg', frame); cap.release(); print('OK' if ret else 'FAILED')"
```

### Test 3: IMU (if connected)
```bash
# Enable I2C
sudo raspi-config
# Interface Options -> I2C -> Enable

# Test I2C detection
i2cdetect -y 1
# Should show device at 0x68 for MPU6050
```

## Safety Considerations

⚠️ **Important Safety Guidelines:**

1. **Emergency Stop**: Always have a way to quickly cut power
2. **Testing**: Test on blocks first (wheels off ground)
3. **Current Limiting**: Use appropriate fuses
4. **Secure Wiring**: Prevent shorts with heat shrink/tape
5. **Stable Mounting**: Secure all components firmly
6. **Battery Safety**: Monitor battery voltage, don't over-discharge
7. **Fire Safety**: Use LiPo battery bags when charging

## Common Wiring Issues

### Motors don't spin
- Check power supply voltage
- Verify motor driver connections
- Test motors with direct battery connection
- Check PWM signals with oscilloscope/multimeter

### Motors spin wrong direction
- Swap motor wire polarity at motor driver output
- Or modify code to invert PWM signals

### Raspberry Pi resets when motors start
- Use separate power supplies
- Add large capacitor (1000µF) across motor power
- Ensure common ground connection

### IMU not detected
- Check I2C is enabled: `sudo raspi-config`
- Verify wiring (SDA, SCL, VCC, GND)
- Run: `i2cdetect -y 1` to scan I2C bus
- Try different I2C pull-up resistor values

## Next Steps

After hardware assembly:

1. Follow [QUICKSTART.md](QUICKSTART.md) for software setup
2. Create hardware motor driver node (see example in QUICKSTART.md)
3. Test motor control with ROS2 topics
4. Tune PID parameters for your specific hardware
5. Calibrate IMU if using balance control

## Bill of Materials (BOM)

Approximate costs (USD):

| Component | Estimated Cost | Source |
|-----------|---------------|--------|
| Raspberry Pi 5 (4GB) | $60 | Official retailers |
| USB Webcam | $15-30 | Amazon, local store |
| Balance Car Chassis | $20-40 | Amazon, AliExpress |
| L298N Motor Driver | $5-10 | Amazon, AliExpress |
| MPU6050 IMU | $5 | Amazon, AliExpress |
| LiPo Battery (3S) | $20-30 | Hobby stores |
| Buck Converter | $5 | Amazon, AliExpress |
| Wires, connectors | $10 | Local electronics store |
| **Total** | **~$150-200** | |

## Recommended Retailers

- **Raspberry Pi**: Official retailers, Adafruit, SparkFun
- **Chassis & Motors**: Amazon, RobotShop, AliExpress
- **Electronics**: Adafruit, SparkFun, Pololu, Digi-Key, Mouser
- **Budget Options**: AliExpress, Banggood (longer shipping)
