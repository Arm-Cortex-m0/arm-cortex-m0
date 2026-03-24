<h1 align="center"> ARM Cortex-M0 CPU Core </h1>

<p align="center">
  <img src="https://img.shields.io/badge/Language-SystemVerilog-00599C?style=for-the-badge" alt="SystemVerilog">
  <img src="https://img.shields.io/badge/ISA-ARMv6--M%20(Thumb)-blue?style=for-the-badge" alt="ARMv6-M">
  <img src="https://img.shields.io/badge/Architecture-5--Stage%20Pipeline-red?style=for-the-badge" alt="Pipeline">
  <img src="https://img.shields.io/badge/Verification-ModelSim-yellow?style=for-the-badge" alt="ModelSim">
</p>

**Key Highlights:**
- **5-Stage Pipelining:** Classic `Fetch`, `Decode`, `Execute`, `Memory`, and `Writeback` stages.
- **Mixed-Width Instruction Decoder:** Custom logic to seamlessly handle the alignment and decoding of 16-bit Thumb and 32-bit instructions without pipeline stalling.

## Hardware Microarchitecture

<img width="2353" height="801" alt="ARM專題架構圖" src="https://github.com/user-attachments/assets/d7711270-e508-46de-b096-c35a1251b3b6" />

### 1. 5-Stage Pipeline Design
- **IF (Instruction Fetch):** Fetches instructions from the Program Memory. Includes PC alignment logic to handle 16-bit boundaries.
- **ID (Instruction Decode):** Detects instruction length (16 vs 32 bits), decodes opcodes, and fetches operands from the Register File.
- **EX (Execute):** Incorporates the ALU and a Barrel Shifter for arithmetic, logical, and shift operations.
- **MEM (Memory):** Handles data memory Read/Write operations.
- **WB (Writeback):** Writes execution results or loaded data back to the Register File.

### 2. Hazard Resolution & Data Forwarding
- Implemented robust **Data Forwarding (Bypassing)** paths from `EX/MEM` and `MEM/WB` stages to the `ID/EX` stage to eliminate data hazard stalls.
- Integrated a **Hazard Detection Unit** to insert pipeline bubbles (NOPs) during Load-Use hazards and branch mispredictions.

## Instruction Set Architecture 
The core implements a highly comprehensive subset of the **ARMv6-M (Thumb & Thumb-2)** ISA, ensuring broad compatibility with standard Cortex-M0 binaries. The implemented instructions include:

- **Data Transfer:** `MOV`
- **Arithmetic Operations:** `ADD`, `ADC`, `SUB`, `SBC`, `RSB`, `MUL`, `CMP`, `CMN`
- **Logical Operations:** `AND`, `ORR`, `EOR`, `BIC`, `MVN`, `TST`
- **Shift Operations:** `ASR`, `LSL`, `LSR`, `ROR`
- **Data Extension & Reversal:** `REV`, `REV16`, `REVSH`, `SXTB`, `SXTH`, `UXTB`, `UXTH`
- **Memory Access:** Supports byte, half-word, and word sizes, along with multi-register operations:
  - Single Data Transfer: `LDR`, `STR`, `LDRH`, `LDRSH`, `STRH`, `LDRB`, `LDRSB`, `STRB`
  - Stack & Multiple Data: `PUSH`, `POP`, `LDM`, `STM`
- **Branch & Control Flow:** `B`, `BX`, `BLX`, and `BL` *(Includes handling for 32-bit Thumb-2 `BL` instructions)*


## RTL Simulation & Verification 

The core logic has been rigorously verified at the Register-Transfer Level (RTL) using **ModelSim**. 

<img width="752" height="137" alt="ARM專題十階乘波形圖" src="https://github.com/user-attachments/assets/cb6ae712-359b-4c08-bee3-b67718ab5b7c" />


- **Complex Algorithm Execution:** Successfully simulated recursive algorithms, including the calculation of **10! (Factorial)**. This proves the robustness of the hardware stack (`PUSH`/`POP`), program counter updates, and iterative branching logic.
- **Pipeline Integrity:** Waveform analysis confirms accurate data forwarding and stall signal generation during `BL` instruction execution and correct Multi-Register memory accesses (`LDM`/`STM`).

---

## Repository Structure

- **`RTL/`** : SystemVerilog source codes for the core microarchitecture (e.g., `mcu.sv`, `ALU.sv`, `Stack.sv`, etc.).
- **`test case text file/`** : Disassembled hex/text files generated via **Keil uVision5**. These contain the compiled machine codes and assembly mnemonics (e.g., `0x00000000 BF00 NOP`) used for testing.
- **`test case/`** : Contains the custom Python script (`asm2sv_.py`) and the auto-generated SystemVerilog instruction memory modules (e.g., `Program_Rom.sv`). The script parses the Keil text files and automatically converts them into synthesizable ROM arrays for ModelSim simulation.

