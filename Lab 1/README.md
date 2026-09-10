# Lab 1: Spin the Romi Wheels

The simplest companion file is [basic_motor_test.py](basic_motor_test.py). Load it on the STM32 NUCLEO-L476RG, then call `left(25)`, `right(25)`, `both(25)`, or `stop()` from the MicroPython REPL. The more complete [motor_control.py](motor_control.py) adds encoder interrupts and the supplied Romi calculations for later lab work.

The assignment-specific files are [lab1_step3_motor_test.py](lab1_step3_motor_test.py), [lab1_step4_motor_spin_test.py](lab1_step4_motor_spin_test.py), [lab1_step5_encoder_test.py](lab1_step5_encoder_test.py), and [lab1_step6_demo.py](lab1_step6_demo.py). They use the `pyb` timer API: step 3 initializes manual motor control, step 4 runs timed forward/reverse motor tests, step 5 reads hardware encoder counts, and step 6 runs a repeatable combined demonstration.

This code assumes MicroPython firmware with `machine.Pin` and `machine.PWM` support. Confirm the firmware's STM32 pin-name syntax before powering the motors. The code starts both channels stopped, uses 20 kHz PWM, and counts encoder transitions with GPIO interrupts. The encoder count is a diagnostic signal; it does not yet close the speed-control loop.

The general system design and its rendered diagram are in [Project Design](../README.md). This document is limited to the Lab 1 motor-driver bring-up.

## Goal

Bring up both motor channels with the NUCLEO-L476RG, command forward and reverse motion, and verify that the wheel direction agrees with the software definition. Encoder feedback is part of the wiring but is not required for the first open-loop spin test.

## Signal map used by the driver

| Function | Left channel | Right channel | Why it is used |
| --- | --- | --- | --- |
| Motor sleep/enable | D2 / PA10 | D5 / PB4 | Independent safety control of each DRV8838 SLP input. Both pins are ordinary digital outputs, so firmware can keep a channel disabled during startup and fault handling. |
| Motor direction | D4 / PB5 | D8 / PA9 | Ordinary digital outputs match the DRV8838 PHASE/DIR-style input. These pins do not consume timer channels needed by PWM or encoders. |
| Motor effort/PWM | D10 / PB6 / TIM4_CH1 | D7 / PA8 / TIM1_CH1 | Hardware timer channels produce stable PWM without software bit-banging. Separate timers avoid a shared-channel conflict and give independent duty cycles. |
| Encoder A | A0 / PA0 / TIM2_CH1 | D12 / PA6 / TIM3_CH1 | Timer input channel 1 provides the first quadrature phase and supports encoder-mode counting. |
| Encoder B | A1 / PA1 / TIM2_CH2 | D11 / PA7 / TIM3_CH2 | Timer input channel 2 provides the second quadrature phase. Pairing A/B on the same timer allows hardware quadrature decoding. |
| Motor power | VIN from Nucleo to VSW | VIN from Nucleo to VSW | The PDB's VSW is switched battery voltage for the motor drivers. It must go to Nucleo VIN, never to 5 V or 3V3. |
| Ground | GND | GND | A shared reference is required for the Nucleo logic signals and the motor-board inputs. |

## Lab 1 hardware datasheets

Use the project [hardware documentation index](../../Documentation/README.md) for the datasheets and manuals. The components directly used in this lab are:

- Pololu Romi Motor Driver and Power Distribution Board, including its board schematic.
- Two TI DRV8838 motor-driver ICs mounted on that board.
- STM32 NUCLEO-L476RG and its STM32L476RG MCU.
- Romi chassis, motors, battery contacts, and Romi quadrature encoder boards.
- Modified Shoe of Brian, documented by the course assembly instructions because it is a course-specific board without a public manufacturer datasheet.

## Why this pinout is appropriate

The DRV8838 exposes simple control inputs: `DIR` selects polarity, `PWM` controls the applied effort, and `SLP` enables or disables the output stage. The map assigns one independent GPIO to each `DIR` and `SLP` signal and one timer-backed output to each `PWM` signal. This gives direct control while preserving predictable startup behavior.

The encoder signals are open-drain quadrature outputs. Each wheel gets both channels on one STM32 timer: TIM2 for the left encoder and TIM3 for the right encoder. This is more robust than counting edges in interrupt callbacks because the timer hardware maintains the count while the CPU performs control work. The selected pins are also adjacent channel pairs on their respective timers.

The reserved I2C pins, D15/PB8 for SCL and D14/PB9 for SDA, are kept free for the BNO055. The Morpho connection is reserved for the Shoe of Brian, which provides the physical stack and USB data interface. The ferrite bead removal means USB must not be treated as a Nucleo power source.

## Lab 1 procedure

1. With batteries removed and power off, inspect every cable against the [project wiring diagram](../romi_nucleo_pin_diagram.drawio).
2. Confirm that the modified Shoe of Brian has its USB 5 V path isolated by the removed ferrite bead.
3. Connect the PDB `VSW` lead to Nucleo `VIN` and connect PDB `GND` to Nucleo `GND`. Do not connect VSW to `5V` or `3V3`.
4. Connect the two motor control cables: left `nSLP-L`, `DIR-L`, `PWM-L`; right `nSLP-R`, `DIR-R`, `PWM-R`.
5. Before installing batteries, use a continuity check to verify ground and inspect for reversed power polarity.
6. Program the Nucleo with both sleep pins driven high, both direction pins initialized low, and both PWM duties set to zero.
7. Raise one PWM duty slowly, first with the robot's drive wheels lifted clear of the table. Verify that only the expected wheel turns.
8. Repeat for the other wheel, then test low-duty forward and reverse commands. If a wheel's positive direction is reversed, invert that side's direction definition in software or swap that motor's two motor leads, but keep encoder A/B labels consistent.
9. Once both motors respond, connect and log encoder A/B counts. Confirm that the count sign agrees with the commanded wheel direction before adding closed-loop control.

## Firmware sequence

```text
initialize GPIO outputs
initialize TIM1_CH1 and TIM4_CH1 for PWM
initialize TIM2 and TIM3 for quadrature encoder mode
set SLP pins high
set DIR pins to the chosen forward state
set PWM duty cycles to 0 percent

for each test command:
    set DIR
    set PWM duty from 0 toward the requested value
    sample and log encoder counts
    stop by setting PWM to 0
```

The PDB documentation states that `PWM = 0` produces dynamic braking while `SLP = 0` places the outputs in coast/high impedance. Lab 1 should use PWM zero for a repeatable stop and SLP low for a deliberate disable or fault response.

## Acceptance checks

- Both wheels respond independently.
- Direction changes only when commanded.
- PWM duty changes motor effort without changing the selected direction.
- Sleep low disables the relevant motor channel.
- Encoder count direction agrees with the software convention.
- No Nucleo reset, brownout, loose connector, or unexpected heating occurs during the test.