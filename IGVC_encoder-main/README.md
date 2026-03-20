# IGVC Dual Encoder Distance & Angle Calculator

## Overview

This project uses the **TM4C123GH6PM (Tiva C LaunchPad)** to read two optical encoders (left & right wheels) using **Wide Timer Module 1** in Capture Mode.  
It calculates each wheel’s **RPM**, **distance traveled**, the robot’s **total distance**, and **heading angle**.

---

## System Info

- **MCU:** TM4C123GH6PM
- **Clock:** 40 MHz
- **Author:** `<Your Name>`
- **Date:** `<Date>`

---

## Hardware Setup

### Left Encoder

- **Pin:** PC6 → WT1CCP0 (Timer 1A, interrupt 112)
- **Mode:** Input Capture (rising edges)

### Right Encoder

- **Pin:** PC7 → WT1CCP1 (Timer 1B, interrupt 113)
- **Mode:** Input Capture (rising edges)

**Other Peripherals**

- **UART0 (PA0/PA1):** Serial output @ 115200 baud
- **RGB LED:** Status indicator

---

## Calculations Explained

Each encoder produces pulses as the wheel rotates.  
The microcontroller measures the **time between pulses** to determine speed and distance.

| Quantity                         | Formula                                                   | Explanation                                                                                      |
| -------------------------------- | --------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| **RPM (Revolutions per Minute)** | `RPM = (SYSCLK * 60) / (ticks * TEETH_PER_REV)`           | Measures how fast the wheel spins. `ticks` is the time (in clock cycles) between encoder pulses. |
| **Wheel Distance**               | `Distance = (edge_count / TEETH_PER_REV) * CIRCUMFERENCE` | Calculates how far one wheel has rolled based on how many encoder teeth passed.                  |
| **Heading Angle (θ)**            | `θ = (RightDist - LeftDist) / Wheelbase`                  | Determines the robot’s change in direction (radians) — if both wheels move equally, θ = 0.       |
| **Forward Distance**             | `Forward = (LeftDist + RightDist) / 2`                    | Finds total distance moved in the forward direction.                                             |

**Constants**

- Encoder teeth per revolution: **18**
- Tire circumference: **45 inches**
- Wheelbase (distance between wheels): **21.89 inches (556 mm)**

---

## Notes

- Timer ticks are measured in **25 ns** units (40 MHz clock).
- Uses **edge-time capture**, up-count mode.
- Timeout detection resets RPM to zero when no pulses are detected.

---
