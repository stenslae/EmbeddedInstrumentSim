# Instrument Simulator for REAL Cubesat — Project Overview

---

## Project Purpose

This project implements an **instrument simulator** running on a Teensy 4.1 microcontroller designed to emulate the behavior of the REAL cubesat instrument for development and testing purposes. It processes incoming command packets from the OBC, and verifies data integrity using a 16-bit CCITT CRC checksum, then outputs telemetry packets using serial communication, allowing robust validation of OBC functioning without using an actual instrument.

---

## Acknowledgements

- This work was done with the Space Science Engineering Lab at MSU, and was largely modified for this specific application.
- **Authors:** Tyler Holliday, Connor Parrott, Andrew Johnson, Emma Stensland, Nevin Leh  

--
