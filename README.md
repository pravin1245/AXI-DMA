# Register-Programmed AXI4 DMA Controller for Streaming Image Pipelines

## Project Overview

This project implements a custom register-programmed AXI4 DMA controller for transferring image data between DDR memory and a hardware image-processing pipeline.

The controller includes independent MM2S and S2MM channels and an AXI4-Lite interface for configuration and status monitoring. The design was verified using SystemVerilog testbenches and validated on a Zynq-7000 FPGA platform with a Gaussian filter.

## Features

- Custom AXI4 DMA controller
- Independent MM2S and S2MM channels
- 32-bit AXI4 memory interface
- 8-bit AXI4-Stream data interface
- AXI4-Lite control and status registers
- 64-word FIFO buffering for each channel
- Maximum burst length of 16 beats
- 4 KB burst-boundary handling
- Support for unaligned memory addresses
- Transfer completion, abort, and reset handling
- Gaussian filter integration
- Layered SystemVerilog verification

## Block Diagram

<img width="1273" height="266" alt="image" src="https://github.com/user-attachments/assets/4c8d4273-472c-48b4-97a1-18017d24d0d6" />


**Data flow:** DDR source buffer → MM2S → Gaussian filter → AXI4-Stream Data FIFO → S2MM → DDR destination buffer.

**Control flow:** The ARM processor configures and monitors the DMA controller through the AXI4-Lite interface.

## Design Specifications

| Parameter | Specification |
|---|---|
| DMA architecture | Register-programmed |
| Memory interface | AXI4, 32-bit |
| Streaming interface | AXI4-Stream, 8-bit |
| Control interface | AXI4-Lite |
| MM2S FIFO depth | 64 words |
| S2MM FIFO depth | 64 words |
| Maximum burst length | 16 beats |
| Transfer length width | 23 bits |
| Image resolution | 256 × 256 pixels |
| Pixel format | 8-bit grayscale |
| Image data size | 65,536 bytes |
| FPGA platform | Zynq-7000 |

## Register Map

| Address Offset | Register | Description |
|---|---|---|
| `0x00` | `MM2S_CR` | MM2S control |
| `0x04` | `MM2S_SR` | MM2S status |
| `0x18` | `MM2S_SA` | Source address |
| `0x28` | `MM2S_LEN` | MM2S transfer length |
| `0x30` | `S2MM_CR` | S2MM control |
| `0x34` | `S2MM_SR` | S2MM status |
| `0x48` | `S2MM_DA` | Destination address |
| `0x58` | `S2MM_LEN` | S2MM transfer length |

**FPGA register base address:** `0x40000000`

The software starts the S2MM channel before starting MM2S. It then polls the status registers to check transfer completion.

## Verification

The design was verified using layered SystemVerilog testbenches with Vivado XSim and Verilator.

Verification covered:

- Register read and write operations
- Different transfer lengths and address offsets
- 4 KB burst-boundary handling
- Stream back-pressure and FIFO stress
- Short and long input streams
- Busy-channel behavior
- Error responses
- Abort and reset operations
- Randomized timing and transfer scenarios
- Reference-memory scoreboard checks
- Injected RTL fault detection

## Simulation and Hardware implementation Results
<img width="968" height="417" alt="image" src="https://github.com/user-attachments/assets/2e010640-4a00-4587-a142-8d29e8351cf9" />

| Parameter | Result |
|---|---:|
| Test items | 295 |
| Clean transfers | 289 |
| Vivado XSim checks | 1,622,060 |
| Checker errors | 0 |
| Functional coverage bins | 46/46 |
| Full-image size | 65,536 bytes |
| Image mismatches in simulation | 0 |
| Injected RTL faults detected | 9/9 |
| Simulation clock | 100 MHz |
| Full-image loop | 65,645 cycles |

The verification results showed no checker errors, and all 46 required functional coverage bins were covered.

## FPGA Validation Results

The DMA controller was integrated with a Gaussian filter and tested using a 256 × 256, 8-bit grayscale image.

| Parameter | Result |
|---|---|
| FPGA platform | Zynq-7000 |
| Image resolution | 256 × 256 |
| Input buffer | `0x18060000` |
| Output buffer | `0x18070000` |
| DMA register base | `0x40000000` |
| MM2S final status | `0x00001002` |
| S2MM final status | `0x00001002` |
| End-to-end processing time | 1.245 ms |
| Average throughput | 52.6 MB/s |

The hardware output was evaluated visually and statistically. Pixel-by-pixel comparison against a software reference was not reported.

## Tools and Technologies

- Verilog and SystemVerilog
- AMD Xilinx Vivado
- Vivado XSim
- Verilator
- AXI4 and AXI4-Stream
- Zynq-7000 FPGA platform
  <img width="1080" height="656" alt="image" src="https://github.com/user-attachments/assets/e6da19cd-9671-4c69-bbe2-ce73782ca69e" />


## Authors
- Pravin Bhangare
- Harshada Dalvi
- Vaishnavi Bauchkar


**Department:** Electronics and Telecommunication Engineering  
**Institute:** Vidyalankar Institute of Technology, Wadala, Mumbai, India


## Future Work

- Pixel-by-pixel comparison with a software Gaussian filter
- Hardware cycle-counter implementation
- Extended verification using assertions and code coverage
- System-level block-design simulation
- Support for interrupts and additional configurations
