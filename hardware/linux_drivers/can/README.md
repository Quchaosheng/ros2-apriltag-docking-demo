# Provisional CAN motor protocol

This directory is the first hardware-independent deliverable for the Linux
driver work. It defines a small, pure-Python codec that can run on Windows,
Linux, CI, or later on a robot computer.

The IDs and fields are provisional:

- Command ID: `0x200 + motor_id`.
- Feedback ID: `0x180 + motor_id`.
- Command payload: signed velocity in mm/s, reserved int16, uint32 sequence.
- Feedback payload: signed position in ticks, signed velocity in mm/s, status,
  and CRC-8/ATM.

`MotorSafetyGate` in `safety.py` provides the first fail-safe policy around the
codec. A command is considered safe only after an MCU heartbeat has been seen
and while both the heartbeat and the most recent command remain fresh. Once
either timeout expires, the adapter must send an explicit stop frame.

Replace these definitions when the MCU protocol is provided. Do not claim this
is the production protocol until hardware and the MCU firmware agree.

Run the tests from this directory:

```text
python -m pytest -q
```
