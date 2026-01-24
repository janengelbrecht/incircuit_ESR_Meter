# In-Circuit ESR Meter

![Status](https://img.shields.io/badge/Status-Complete-green)
![License](https://img.shields.io/badge/License-MIT-blue)
![Version](https://img.shields.io/badge/Version-1.00-orange)
![ESP32](https://img.shields.io/badge/ESP32-WROOM--32-blue)

## What is an ESR Meter?

An ESR meter is a specialized two-terminal instrument designed to measure the equivalent series resistance (ESR) of real-world capacitors. One of its key advantages is that it often allows testing with the capacitor still in circuit.

### Why ESR Matters

Aluminum electrolytic capacitors naturally exhibit higher ESR, which worsens over time due to aging, heat, and ripple current. This degradation can lead to equipment malfunctions. In older devices, electrolytics can fail mechanically (bulging or leakage), but visual inspection isn’t foolproof—capacitors that look fine can still have degraded ESR.

### How Accurate Does It Need to Be?

For troubleshooting, precise ESR values are rarely necessary; any functional meter will usually suffice. When accuracy is critical, measurements must be taken under controlled conditions because ESR varies with temperature, frequency, and applied voltage.

## Measuring ESR

The basic principle involves applying a short current pulse or a high-frequency AC signal where the capacitor’s reactance is negligible, and measuring the resulting voltage drop across the ESR. Professional ESR meters simplify the process, enabling rapid testing of multiple components. Instruments like LCR bridges can also measure ESR accurately, but dedicated ESR meters are inexpensive and optimized for quick troubleshooting.

## How ESR Meters Work

Most ESR meters briefly discharge the capacitor and send a short current pulse through it. Because the pulse is too short for significant charging, the resulting voltage reflects the ESR (plus a negligible reactive component). Alternatively, some meters use high-frequency AC signals so that capacitive reactance is minimal compared to ESR. Circuit designs typically target capacitors of 1 µF or larger, covering common electrolytics.

## Interpreting Results

Acceptable ESR depends on capacitance—larger capacitors generally have lower ESR. Values can be checked against typical charts or compared to new components. Manufacturer specifications exist but may not reflect in-circuit conditions. Because ESR is far lower than other parallel resistances in a circuit, in-circuit testing is possible. ESR meters use low voltages to avoid activating semiconductor junctions, which could distort readings.

## Limitations

- ESR meters do not measure capacitance; that requires a capacitance meter.
- A shorted capacitor will appear ideal on an ESR meter, so an ohmmeter check is recommended.
- ESR can vary with operating conditions; a capacitor may test fine when cold but fail under load.
- Connecting an ESR meter to a charged or live capacitor can damage the meter, though protective diodes help mitigate this risk.
- Inductive components distort readings, so ESR meters cannot measure transformer windings or similar parts.

## Other Applications

An ESR meter can function as a pulsed or high-frequency AC milliohmmeter. It can measure low resistances such as battery internal resistance, switch contacts, or PCB trace segments.

## Project Overview

The In-Circuit ESR Meter is a professional-grade, open-source design for safe in-circuit testing of electrolytic capacitors. This project includes hardware, firmware, and documentation.

### Key Features

- Safe In-Circuit Testing: ≤0.1 V test signal prevents semiconductor activation  
- High Precision: 1 mΩ resolution, ≤5% accuracy over 0–10 Ω range  
- Professional Design: 4-wire Kelvin connections, EMI/EMC/ESD protection  
- Dual Interface: LCD display + SCPI command protocol for automation  
- Robust Operation: Hardware watchdog, error recovery, oversampling

## Hardware Design

### Core Components

- MCU: ESP32-WROOM-32 (WiFi/Bluetooth capable)  
- DDS Generator: AD9850 for a 100 kHz sine wave  
- ADC: ADS1115 16-bit ΔΣ ADC with programmable gain (4×, 8×, 16×)  
- OpAmps: TS3V902 rail-to-rail CMOS operational amplifiers  
- Display: 16×2 LCD with I2C backpack  
- Power: USB-C input with +3.3V LDO and -5V switched-capacitor converter

### Circuit Architecture

- Signal Generation: AD9850 DDS → Buffer (TS3V902 voltage follower)  
- Measurement Circuit: 10 µF coupling capacitor + 10 Ω precision reference resistor  
- Differential Amplification: 47× gain differential amplifier (TS3V902)  
- Rectification: Precision superdiode full-wave rectifier (2× TS3V902 + Schottky diodes)  
- Filtering: 3rd-order Sallen–Key Butterworth low-pass filter (100 Hz cutoff)  
- ADC Protection: Schottky clamping diodes before ADS1115 input  
- Kelvin Connections: 4-pin isolated probe connections

### PCB Design

- 2-layer FR-4, 1.6 mm thickness  
- Complete ground planes on both sides  
- EMI guarding around analog signals  
- ESD protection on all external connections  
- Analog/digital separation with ≥5 mm spacing

## Firmware

### Architecture

- Language: Arduino C++ (C++14 compatible)  
- Paradigm: Structured programming (no classes)  
- Memory: Static allocation only (no dynamic allocation)  
- Structure: Single .ino file with modular organization

### Key Modules

- HAL (Hardware Abstraction Layer): AD9850, ADS1115, LCD, buttons  
- ADC Processing: 64× oversampling, automatic PGA range selection  
- ESR Calculation: Table-based linear interpolation with PROGMEM storage  
- State Machine: Deterministic system flow control  
- SCPI Interface: Standard instrument protocol for remote control (basic commands implemented)  
- Watchdog: 3-second timeout with automatic recovery

### SCPI Commands Supported

- *IDN? — Identification query  
- MEASure:ESR? — Perform ESR measurement  
- SYSTem:PRESet — System reset to defaults  
- DISPlay:MODE <RAW|ESR> — Set display mode  
- SYSTem:COMMunication <ON|OFF> — Enable/disable SCPI mode  
- *TST? — Self-test  
- SYSTem:VERSION? — Firmware version  
- SYSTem:HELP — List available commands

## Documentation Structure

- Line-by-line comments: Every code line has explanatory comments  
- Doxygen formatting: Professional function documentation  
- Hardware netlist: Complete component listing with connections  
- Test procedures: Validation and calibration instructions  
- Developer guide: Architectural overview and enhancement roadmap

## Getting Started

### Prerequisites

- Hardware: All components from netlist (see hardware report)  
- Software: Arduino IDE 2.x or PlatformIO  
- Libraries: Wire, Adafruit_ADS1X15, LiquidCrystal_I2C  
- ESP32 Board Support: ESP32 Arduino package

### Building & Flashing

1. Install required libraries via the Arduino Library Manager  
2. Select board: ESP32 Wrover Module  
3. Set upload speed to 921600  
4. Open `ESR_Meter.ino` and upload to ESP32  
5. Connect via serial monitor at 115200 baud

## Calibration

Important: The firmware requires calibration before first use!

- Prepare precision reference resistors (0.1 Ω to 10 Ω)  
- Run emergency calibration routine via SCPI or serial command  
- Measure actual ADC values for each known resistance  
- Update PROGMEM tables in firmware with calibration data  
- Re-flash firmware with calibrated tables

_Note:_ See the Firmware/Calibration section for exact calibration procedure and sample scripts.

## Testing & Validation

### Performance Specifications

- ESR Range: 0–10 Ω  
- Resolution: 1 mΩ  
- Accuracy: ≤5% of reading  
- Test Frequency: 100 kHz ±100 ppm  
- Test Signal: ≤0.1 V p-p at probes  
- Measurement Time: ≤100 ms  
- Power Consumption: ~280 mA typical

### Validation Tests

- Precision Test: Measure known resistors (0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0, 10.0 Ω)  
- Resolution Test: Verify 1 mΩ distinction capability  
- Safety Test: Confirm ≤0.1 V p-p signal at test points  
- Robustness Test: ESD and EMI immunity

## Use Cases

1. Hobbyist Electronics Repair — Troubleshoot faulty electrolytic capacitors, verify new caps before installation  
2. Professional Service Workshop — Quick diagnosis of power supplies, verification of repairs  
3. Educational Applications — Lab exercises on capacitor properties and measurement principles  
4. Research & Development — Characterization, lifetime testing, production test development

## Project Status & Roadmap

### Current Status (v1.00)

- Hardware design and netlist: Complete  
- Firmware: Fully functional with core features  
- SCPI: Basic implementation available  
- Documentation: Comprehensive

### Known Limitations

- ESR tables require firmware re-flash for calibration updates  
- SCPI implementation is partial (basic commands only)  
- No persistent storage for configuration/calibration (current firmware requires reflashing for table changes)  
- Monolithic file structure limits scalability

### Recommended Enhancements

- Persistent storage: Save calibration data to ESP32 NVS  
- Zero calibration via device UI (menu) to allow recalibration without PC  
- Temperature compensation: Activate and refine real-time correction  
- Modular firmware: Split into .h/.cpp files for maintainability  
- Data logging and improved SCPI: For production/automation use

## Analysis & Recommendations (Calibration & Reliability)

### Summary

- The largest real-world source of measurement error is probe contact resistance and mechanical wear. This can change quicker than component drift and is the primary reason in-field recalibration is necessary.  
- ADC and op-amp drift occur slowly over years and are usually within acceptable limits for hobby use, but probe wear can require recalibration every 6–12 months in heavy use.

### Critical Recommendation

Add a "Zero Calibration" menu item so technicians can short the probes, press a button, and save the zero offset in NVS. This greatly improves practical usability for service environments.

Example code snippet (Arduino-style):
```cpp
void menu_zero_calibration() {
  lcd.clear();
  lcd.print("Short circuit probes");
  lcd.setCursor(0,1);
  lcd.print("Press OK");
  while(!button_pressed) { delay(10); }
  int32_t zero_offset = adc_read_oversampled(PGA_2X_INDEX);
  preferences.begin("esr-cal", false);
  preferences.putInt("zero_offset", zero_offset);
  preferences.end();
  lcd.clear();
  lcd.print("Calibrated!");
}
```

This change can be implemented in ~2 hours of firmware work and will solve the main practical problem for service workshops.

## Safety & Maintenance

### Critical Warnings

- ALWAYS power off the circuit under test before connecting probes  
- VERIFY test signal is ≤0.1 V p-p before in-circuit measurements  
- AVOID testing in high-voltage circuits (>50 V)  
- USE ESD precautions when handling the instrument

### Maintenance Schedule

- Monthly: Check probe condition and test with known references  
- Every 6 months: Recalibrate with precision resistors (or use zero-cal feature)  
- Annually: Full system check and PCB inspection for corrosion

## License

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

## Acknowledgments

- Component manufacturers: Analog Devices (AD9850), Texas Instruments (ADS1115, TS3V902), Espressif (ESP32)  
- Open-source community and reference designs

