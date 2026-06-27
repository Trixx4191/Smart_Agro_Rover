# 🌱 AUTONOMOUS FARM ROVER — Bill of Materials (Phase 3)

Target Budget: $100–$200 USD | Market: US (Amazon / AliExpress / Adafruit / SparkFun)

## BOM

### COMPUTE

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 1 | Compute | Single-Board Computer | Raspberry Pi 4 Model B (2GB) | 1 | 45 | 45 | MUST HAVE | CanaKit / Amazon | Brain of rover – runs Python, vision, navigation |
| 2 | Compute | MicroSD Card | SanDisk 32GB Class 10 | 1 | 8 | 8 | MUST HAVE | Amazon | OS + data storage for Pi |
| 3 | Compute | Microcontroller | Arduino Nano (ATmega328P) | 1 | 5 | 5 | MUST HAVE | Amazon / AliExpress | Reflex layer – sensors & motor control |

### SENSORS

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 4 | Soil Sensors | Capacitive Soil Moisture Sensor | Capacitive v1.2 (analog) | 2 | 2.5 | 5 | MUST HAVE | AliExpress | More reliable than resistive; 2 for redundancy |
| 5 | Soil Sensors | Waterproof Temp Sensor | DS18B20 (1-Wire) | 2 | 2 | 4 | MUST HAVE | Amazon / AliExpress | Soil temperature, 1-Wire protocol |
| 6 | Environment | Temp/Humidity/Pressure | BME280 (I2C breakout) | 1 | 5 | 5 | MUST HAVE | Adafruit / Amazon | Air temp, humidity, altitude |

### VISION

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 7 | Vision | Camera Module | Raspberry Pi Camera Module v2 | 1 | 25 | 25 | MUST HAVE | Amazon / PiShop | 8MP; native CSI interface to Pi |
| 8 | Vision | Camera Flex Cable (extended) | 50cm Pi Camera Ribbon Cable | 1 | 2 | 2 | NICE TO HAVE | Amazon | Allows camera mast mounting |

### NAVIGATION

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 9 | Navigation | GPS Module | NEO-6M GPS Module (UART) | 1 | 7 | 7 | MUST HAVE | Amazon / AliExpress | Waypoint navigation; ~2.5m accuracy |
| 10 | Navigation | IMU (Accel + Gyro + Mag) | MPU-6050 (I2C) | 1 | 3 | 3 | MUST HAVE | Amazon / AliExpress | Heading + tilt detection |
| 11 | Navigation | Ultrasonic Sensor | HC-SR04 | 2 | 1.5 | 3 | MUST HAVE | Amazon | Front + rear obstacle detection |

### MOBILITY

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 12 | Mobility | 4WD Robot Chassis Kit | 4WD Smart Car Chassis + DC motors | 1 | 18 | 18 | MUST HAVE | Amazon / AliExpress | Includes motors, wheels, frame, screws |
| 13 | Mobility | Motor Driver | L298N Dual H-Bridge Module | 1 | 4 | 4 | MUST HAVE | Amazon | Controls 4 DC motors (2 per channel) |
| 14 | Mobility | Wheel Encoders | LM393 Speed Sensor (slot type) | 4 | 1 | 4 | NICE TO HAVE | AliExpress | Odometry for dead-reckoning nav |

### POWER

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 15 | Power | LiPo Battery Pack | 3S 5200mAh 11.1V LiPo | 1 | 22 | 22 | MUST HAVE | Amazon / HobbyKing | Main power source; ~3–4hr runtime estimate |
| 16 | Power | Buck Converter (5V) | LM2596 Step-Down Module | 2 | 1.5 | 3 | MUST HAVE | Amazon / AliExpress | 5V for Pi + Arduino |
| 17 | Power | Buck Converter (3.3V) | AMS1117 3.3V Module | 1 | 0.8 | 0.8 | MUST HAVE | AliExpress | 3.3V for sensors |
| 18 | Power | Power Switch | Rocker Switch (20A rated) | 1 | 1.5 | 1.5 | MUST HAVE | Amazon | Main kill switch |
| 19 | Power | Inline Fuse Holder + Fuses | 5A & 10A fuses | 1 | 2.5 | 2.5 | MUST HAVE | Amazon | Safety – protects battery circuit |

### COMMS & WIRING

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 20 | Communication | USB A to Mini-B Cable | Pi ↔ Arduino Serial Link | 1 | 2 | 2 | MUST HAVE | Amazon | Serial comms between Pi and Arduino |
| 21 | Wiring | Jumper Wire Kit | M-M / M-F / F-F 120pcs | 1 | 5 | 5 | MUST HAVE | Amazon | Prototyping connections |
| 22 | Wiring | Breadboard | 400-tie Mini Breadboard | 1 | 2 | 2 | MUST HAVE | Amazon | Sensor/circuit prototyping |
| 23 | Wiring | PCB Prototype Board | Double-sided 5x7cm | 2 | 1.5 | 3 | NICE TO HAVE | AliExpress | For permanent sensor wiring |
| 24 | Wiring | Heat Shrink Tubing Kit | Assorted sizes | 1 | 3 | 3 | NICE TO HAVE | Amazon | Wire insulation & weather protection |

### ENCLOSURE & MISC

| # | Subsystem | Component / Part | Model / Spec | Qty | Unit Price ($) | Total ($) | Priority | Source | Notes |
|---|---|---|---|---:|---:|---:|---|---|---|
| 25 | Enclosure | Waterproof Project Box | IP65 ABS Box ~200x120mm | 1 | 8 | 8 | MUST HAVE | Amazon | Houses Pi, Arduino, power electronics |
| 26 | Misc | M3 Screw/Standoff Kit | M3 Brass Standoff Assortment | 1 | 6 | 6 | MUST HAVE | Amazon | Mounting boards and components |
| 27 | Misc | Cable Ties + Velcro Strips | Assorted pack | 1 | 3 | 3 | NICE TO HAVE | Amazon | Cable management |

| Summary | Total ($) |
|---|---:|
| TOTAL ESTIMATED COST | 199.8 |
| MUST HAVE items only | 184.8 |

> 💡 Blue = Hardcoded input (you can edit) | Black = Calculated formula | Priority: MUST HAVE = core build, NICE TO HAVE = adds capability

## Budget Summary

All values linked from BOM sheet — edit unit prices there.

| Category | Subtotal ($) | % of Budget | Notes |
|---|---:|---:|---|
| Compute | 53 | 0 |  |
| Sensors | 9 | 0 |  |
| Vision | 25 | 0 |  |
| Navigation | 10 | 0 |  |
| Mobility | 22 | 0 |  |
| Power | 27.3 | 0 |  |
| Comms & Wiring | 12 | 0 |  |
| Enclosure & Misc | 14 | 0 |  |
| GRAND TOTAL | 172.3 | 100.0% |  |

### Budget Status

| Item | Value |
|---|---:|
| Target Budget | $200.00 |
| Total Cost | 172.3 |
| Remaining / Overage | 27.7 |
| Status | ✅ Within Budget |

## Procurement Checklist

| ✓ | Component | Source | Qty | Est. Price ($) | Status | Link / Notes |
|---|---|---|---:|---:|---|---|
| ☐ | Raspberry Pi 4 (2GB) | Amazon / CanaKit | 1 | 45 | Not Ordered |  |
| ☐ | SanDisk 32GB MicroSD | Amazon | 1 | 8 | Not Ordered |  |
| ☐ | Arduino Nano | Amazon / AliExpress | 1 | 5 | Not Ordered |  |
| ☐ | Capacitive Moisture Sensor x2 | AliExpress | 2 | 2.5 | Not Ordered |  |
| ☐ | DS18B20 Temp Sensor x2 | Amazon / AliExpress | 2 | 2 | Not Ordered |  |
| ☐ | BME280 Module | Adafruit / Amazon | 1 | 5 | Not Ordered |  |
| ☐ | Pi Camera Module v2 | Amazon / PiShop | 1 | 25 | Not Ordered |  |
| ☐ | Camera Ribbon 50cm | Amazon | 1 | 2 | Not Ordered |  |
| ☐ | NEO-6M GPS Module | Amazon / AliExpress | 1 | 7 | Not Ordered |  |
| ☐ | MPU-6050 IMU | Amazon / AliExpress | 1 | 3 | Not Ordered |  |
| ☐ | HC-SR04 Ultrasonic x2 | Amazon | 2 | 1.5 | Not Ordered |  |
| ☐ | 4WD Chassis Kit | Amazon / AliExpress | 1 | 18 | Not Ordered |  |
| ☐ | L298N Motor Driver | Amazon | 1 | 4 | Not Ordered |  |
| ☐ | LM393 Wheel Encoder x4 | AliExpress | 4 | 1 | Not Ordered |  |
| ☐ | 3S 5200mAh LiPo Battery | Amazon / HobbyKing | 1 | 22 | Not Ordered |  |
| ☐ | LM2596 Buck 5V x2 | AliExpress | 2 | 1.5 | Not Ordered |  |
| ☐ | AMS1117 3.3V Module | AliExpress | 1 | 0.8 | Not Ordered |  |
| ☐ | Rocker Switch | Amazon | 1 | 1.5 | Not Ordered |  |
| ☐ | Fuse Holder + Fuses | Amazon | 1 | 2.5 | Not Ordered |  |
| ☐ | USB A-MiniB Cable | Amazon | 1 | 2 | Not Ordered |  |
| ☐ | Jumper Wire Kit | Amazon | 1 | 5 | Not Ordered |  |
| ☐ | Mini Breadboard | Amazon | 1 | 2 | Not Ordered |  |
| ☐ | PCB Prototype Board x2 | AliExpress | 2 | 1.5 | Not Ordered |  |
| ☐ | Heat Shrink Kit | Amazon | 1 | 3 | Not Ordered |  |
| ☐ | IP65 Waterproof Box | Amazon | 1 | 8 | Not Ordered |  |
| ☐ | M3 Standoff/Screw Kit | Amazon | 1 | 6 | Not Ordered |  |
| ☐ | Cable Ties + Velcro | Amazon | 1 | 3 | Not Ordered |  |
