What is an ESR Meter?
An ESR meter is a specialized two-terminal instrument designed to measure the equivalent series resistance (ESR) of real-world capacitors. One of its key advantages is that it often allows testing without removing the capacitor from the circuit. Standard tools like capacitance meters cannot measure ESR, although some combined devices can check both ESR and out-of-circuit capacitance. Conventional DC milliohmmeters or multimeters are unsuitable because they rely on steady direct current, which cannot pass through a capacitor. Most ESR meters can also measure very low, non-inductive resistances, making them useful for additional tasks beyond capacitor testing.

Why ESR Matters

Aluminum electrolytic capacitors naturally exhibit higher ESR, which worsens over time due to aging, heat, and ripple current. This degradation can lead to equipment malfunctions. In older devices, symptoms included hum and poor performance, while modern systems—especially switch-mode power supplies—are extremely sensitive to ESR. A capacitor with excessive ESR can cause voltage instability, system failure, or even permanent damage. Despite these risks, electrolytic capacitors remain popular because they offer high capacitance at low cost and compact size, typically ranging from a few microfarads to tens of thousands.
Faulty capacitors often show physical signs like bulging or leakage as internal chemicals break down into gas. However, visual inspection isn’t foolproof—capacitors that look fine can still have dangerously high ESR, detectable only through measurement.

How Accurate Does It Need to Be?

For troubleshooting, precise ESR values are rarely necessary; any functional meter will usually suffice. When accuracy is critical, measurements must be taken under controlled conditions because ESR varies with frequency, voltage, and temperature. General-purpose ESR meters, which operate at fixed frequencies, are not suitable for high-precision lab work.

Measuring ESR

The basic principle involves applying an AC signal at a frequency where the capacitor’s reactance is negligible, often using a voltage divider setup. Quick checks can be done with improvised tools—a square-wave generator and oscilloscope, or a sine-wave source with an AC voltmeter—comparing results against a known good capacitor.
Professional ESR meters simplify the process, enabling rapid testing of multiple components. Instruments like LCR bridges or Q meters can also measure ESR accurately, but dedicated ESR meters are inexpensive, single-purpose devices focused on identifying capacitors with unacceptable resistance.

How ESR Meters Work

Most ESR meters briefly discharge the capacitor and send a short current pulse through it. Because the pulse is too short for significant charging, the resulting voltage reflects the ESR (plus a negligible capacitive effect). The meter calculates resistance by dividing voltage by current and displays the result in ohms or milliohms. This process repeats thousands of times per second.
Alternatively, some meters use high-frequency AC signals so that capacitive reactance is minimal compared to ESR. Circuit design typically targets capacitors of one microfarad or larger, covering common aluminum electrolytics prone to ESR issues.

Interpreting Results

Acceptable ESR depends on capacitance—larger capacitors generally have lower ESR. Values can be checked against typical charts or compared to new components. Manufacturer specifications exist but are rarely needed. In practice, ESR problems escalate quickly: once resistance starts rising, it often jumps from acceptable to clearly faulty. For large capacitors, anything above a few ohms is usually unacceptable.
Because ESR is far lower than other parallel resistances in a circuit, in-circuit testing is possible. ESR meters use low voltages to avoid activating semiconductor junctions, which could distort readings.

Limitations


ESR meters do not measure capacitance; that requires a separate capacitance meter.
A shorted capacitor will appear ideal on an ESR meter, so an ohmmeter check is recommended.
ESR can vary with operating conditions; a capacitor may test fine when cold but fail under load.
Connecting an ESR meter to a charged or live capacitor can damage the meter, though protective diodes help mitigate this risk.
Inductive components distort readings, so ESR meters cannot measure transformer windings or similar parts.


Other Applications

Essentially, an ESR meter functions as a pulsed or high-frequency AC milliohmmeter. It can measure low resistances such as battery internal resistance, switch contacts, or PCB trace segments. Its low test voltage prevents false readings caused by semiconductor junctions, making it useful for locating short circuits—even among parallel components. Tweezer-style probes are handy for densely packed boards, especially with surface-mount technology.

In-Circuit ESR Meter

A professional-grade, open-source ESR (Equivalent Series Resistance) meter designed for safe in-circuit testing of electrolytic capacitors. This project includes complete hardware, firmware, and documentation for building a precision measurement instrument suitable for hobbyists, technicians, and educational use.

https://img.shields.io/badge/Status-Complete-green
https://img.shields.io/badge/License-Proprietary-blue
https://img.shields.io/badge/Version-1.00-orange
https://img.shields.io/badge/ESP32-WROOM--32-blue

📋 Project Overview

The In-Circuit ESR Meter measures the equivalent series resistance of electrolytic capacitors (0-10 Ω range) with 1 mΩ resolution and ≤5% accuracy using a 100 kHz test signal limited to 0.1V p-p for safe in-circuit operation. The system implements 4-wire Kelvin measurement techniques, automatic PGA (Programmable Gain Amplifier) ranging, and comprehensive error handling.

Key Features

Safe In-Circuit Testing: ≤0.1V test signal prevents semiconductor activation
High Precision: 1 mΩ resolution, ≤5% accuracy over 0-10 Ω range
Professional Design: 4-wire Kelvin connections, EMI/EMC/ESD protection
Dual Interface: LCD display + SCPI command protocol for automation
Robust Operation: Hardware watchdog, error recovery, oversampling

🔧 Hardware Design

Core Components

MCU: ESP32-WROOM-32 (WiFi/Bluetooth capable)
DDS Generator: AD9850 for precise 100 kHz sine wave
ADC: ADS1115 16-bit ΔΣ ADC with programmable gain (4×, 8×, 16×)
OpAmps: TS3V902 rail-to-rail CMOS operational amplifiers
Display: 16×2 LCD with I2C backpack
Power: USB-C input with +3.3V LDO and -5V switched capacitor converter

Circuit Architecture

Signal Generation: AD9850 DDS → Buffer (TS3V902 voltage follower)
Measurement Circuit: 10 μF coupling capacitor + 10 Ω precision reference resistor
Differential Amplification: 47× gain differential amplifier (TS3V902)
Rectification: Precision superdiode full-wave rectifier (2× TS3V902 + Schottky diodes)
Filtering: 3rd-order Sallen-Key Butterworth low-pass filter (100 Hz cutoff)
ADC Protection: Schottky clamping diodes before ADS1115 input
Kelvin Connections: 4-pin isolated probe connections

PCB Design

2-layer FR-4, 1.6 mm thickness
Complete ground planes on both sides
EMI guarding around analog signals
ESD protection on all external connections
Analog/digital separation with 5 mm minimum spacing

💻 Firmware

Architecture

Language: Arduino C++ (C++14 compatible)
Paradigm: Structured programming (no classes)
Memory: Static allocation only (no malloc/free)
Structure: Single .ino file with modular organization

Key Modules

HAL (Hardware Abstraction Layer): AD9850, ADS1115, LCD, buttons
ADC Processing: 64× oversampling, automatic PGA range selection
ESR Calculation: Table-based linear interpolation with PROGMEM storage
State Machine: Deterministic system flow control
SCPI Interface: Standard instrument protocol for remote control
Watchdog: 3-second timeout with automatic recovery

SCPI Commands Supported
*IDN? - Identification query
MEASure:ESR? - Perform ESR measurement
SYSTem:PRESet - System reset to defaults
DISPlay:MODE <RAW|ESR> - Set display mode
SYSTem:COMMunication <ON|OFF> - Enable/disable SCPI mode
*TST? - Self-test
SYSTem:VERSION? - Firmware version
SYSTem:HELP - List available commands

📁 Documentation Structure

Documentation Features

Line-by-line comments: Every code line has explanatory comments
Doxygen formatting: Professional function documentation
Hardware netlist: Complete component listing with connections
Test procedures: Validation and calibration instructions
Developer guide: Architectural overview and enhancement roadmap

🚀 Getting Started

Prerequisites

Hardware: All components from netlist (see hardware report)
Software: Arduino IDE 2.x or PlatformIO
Libraries: Wire, Adafruit_ADS1X15, LiquidCrystal_I2C
ESP32 Board Support: ESP32 Arduino package

Building & Flashing

Install required libraries via Arduino Library Manager
Select board: ESP32 Wrover Module
Set upload speed to 921600
Open ESR_Meter.ino and upload to ESP32
Connect via serial monitor at 115200 baud

Calibration

Important: The firmware requires calibration before first use!
Prepare precision reference resistors (0.1 Ω to 10 Ω)
Run emergency calibration routine via SCPI or serial command
Measure actual ADC values for each known resistance
Update PROGMEM tables in firmware with calibration data
Re-flash firmware with calibrated tables

🧪 Testing & Validation

Performance Specifications

ESR Range: 0-10 Ω
Resolution: 1 mΩ
Accuracy: ≤5% of reading
Test Frequency: 100 kHz ±100 ppm
Test Signal: ≤0.1V p-p at probes
Measurement Time: ≤100 ms
Power Consumption: ~280 mA typical

Validation Tests

Precision Test: Measure known resistors (0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0, 10.0 Ω)
Resolution Test: Verify 1 mΩ distinction capability
Safety Test: Confirm ≤0.1V p-p signal at test points
Robustness Test: ESD and EMI immunity

👥 Use Cases

1. Hobbyist Electronics Repair

Troubleshoot faulty electrolytic capacitors in PCBs
Verify new capacitors before installation
Periodic maintenance of audio equipment and power supplies

2. Professional Service Workshop

Quick diagnosis of power supply units
Verification of repair work
Documentation for customer reports

3. Educational Applications
Laboratory exercises on capacitor properties
Demonstration of ESR concept and practical implications
Student projects on measurement principles

4. Research & Development
Characterization of new capacitor technologies
Lifetime testing and aging experiments
Development of production test procedures

🔍 Project Status & Roadmap

Current Status (v1.00)

Complete hardware design and netlist
Fully functional firmware with core features
Basic SCPI protocol implementation
Comprehensive documentation

Known Limitations

ESR tables require firmware re-flash for calibration updates
SCPI implementation is partial (basic commands only)
No persistent storage for configuration/calibration
Monolithic file structure limits scalability

Future Enhancements
Persistent Storage: Save calibration data to ESP32 NVS
Full SCPI Implementation: Complete command set with error queues
Modular Architecture: Split firmware into separate .h/.cpp files
Enhanced UI: Menu system with status indicators
Temperature Compensation: Real-time ESR correction
Data Logging: SD card support for measurement records

⚠️ Safety & Usage Notes

Critical Warnings

ALWAYS power off the circuit under test before connecting probes
VERIFY test signal is ≤0.1V p-p before in-circuit measurements
AVOID testing in high-voltage circuits (>50V)
USE ESD precautions when handling the instrument

Maintenance

Monthly: Check physical probe connections, test with known references
Every 6 months: Recalibrate with precision reference resistors
Annually: Full system test, inspect PCB for corrosion

📝 License & Attribution

Copyright © 2026 Jan Engelbrecht Pedersen
All rights reserved.

The software is provided "as is", without warranty of any kind. Commercial use requires permission from the author.

🙏 Acknowledgments

Reference Design: Based on established ESR measurement principles
Component Manufacturers: Analog Devices (AD9850), Texas Instruments (ADS1115, TS3V902), Espressif (ESP32)
Documentation Standards: Inspired by professional instrumentation documentation practices
For questions, issues, or contributions, please refer to the detailed developer guide in the firmware comments.

The project includes hardware, firmware, and documentation for a professional-grade ESR meter.
We are to write in English and assume the project is under MIT License.


📄 License
MIT License

Copyright (c) 2026 Jan Engelbrecht Pedersen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

🙏 Acknowledgments
Component manufacturers for high-quality parts and documentation

Open-source community for inspiration and reference designs

Test equipment manufacturers whose instruments provided validation references
