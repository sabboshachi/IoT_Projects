# UART Protocol Communication

## What is UART Protocol?
UART stands for Universal Asynchronous Receiver/Transmitter communication protocol used for serial data communication between two devices.

The main features of UART Protocol are:
1. Asynchronous Communication: 
    * Baud rate (e.g., 9600 bps)
    * Number of data bits (e.g., 8 bits)
    * Parity bit (optional, for error checking)
    * Stop bits (e.g., 1 or 2 bits) 
2. Data Transmission:
    * Start bit: Indicates the beginning of data.
    * Data bits: The actual information, typically 7, 8, or 9 bits.
    * Parity bit: Optional, helps detect errors in transmission.
    * Stop bit(s): Indicates the end of a data frame.
3. Full Duplex Communication:
    * UART can send and receive data simultaneously because it   has separate lines for transmitting (TX) and receiving (RX)."

## Which Secnario UART Protocol is used for?
1. Microcontroller-to-PC Communication
2. Microcontroller-to-Microcontroller Communication
3. Communication with Sensors or Modules
4. Debugging in Embedded Systems
5. Industrial Automation
6. IoT Devices
7. Bootloaders or Firmware Updates
8. Simple Robotics and Automation
9. Low-Power, Short-Distance Communication

## Why Use UART in these Scenarios?
* Simplicity: UART requires minimal setup and hardware.
* Low cost: Only two data lines (TX and RX) are needed.
* Point-to-point communication: Best for one-to-one communication, which is sufficient in many applications."

## Uart Communication Diagram or Frame Format?

| Start   |      Data (7-9 bits)      | Parity    | Stop  |
|    0    |   D0 D1 ... D7 (or 9)     |  (P)      | 1/2   |

## Hardware Overview
* TX (Transmit): Sends data to another UART.
* RX (Receive): Receives data from another UART.
* Communication occurs over two wires (TX and RX).

## UART Configuration and Setting Parameter
* works for Simplex, Half Duplex, Full Duplex
* Buad Rate / Bit Rate = Rate of Data transfer in serial communication expressed in bit per second (2400,4800,9600 etc)
* Start and Stop Bit
