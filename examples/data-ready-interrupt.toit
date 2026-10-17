// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import bmm350 show Bmm350

/**
Reads the BMM350 each time its INT pin reports a new measurement.

Wire the chip's INT pin to the ESP32 pin named in $INT-PIN. The program turns
  on the data ready signal, then waits for the pin to go high instead of asking
  the chip over I2C whether a sample is ready. That leaves the processor free
  between samples.

The pin is set up as push-pull, active high and latched. Latched means that the
  pin stays high until the chip's status is read, and reading a sample reads
  the status. A short pulse could pass unseen between two checks of the pin,
  and a latched signal cannot.
*/

SDA-PIN ::= 21
SCL-PIN ::= 20
INT-PIN ::= 7

// Choices made for this example. A slow rate keeps the output readable.
RATE ::= Bmm350.ODR-25HZ
READINGS ::= 50
TIMEOUT ::= Duration --s=2

main:
  bus := i2c.Bus
    --sda=SDA-PIN
    --scl=SCL-PIN
    --frequency=400_000

  if not (bus.test Bmm350.I2C-ADDRESS):
    print "No BMM350 found on the bus. Check the wiring and the I2C address."
    return

  sensor := Bmm350 (bus.device Bmm350.I2C-ADDRESS)
  sensor.configure-mag --odr=RATE --avg=Bmm350.AVG-NONE

  // Report data ready on the INT pin.
  sensor.set-data-ready-check true
  sensor.set-int-pin-push-pull true
  sensor.set-int-pin-polarity true
  sensor.set-int-latched-mode true
  sensor.set-int-pin true

  // The pull-down keeps the input low if the wire comes loose.
  interrupt := gpio.Pin.in INT-PIN --pull-down

  count := 0
  first-us := 0
  last-us := 0
  while count < READINGS:
    timed-out := catch --unwind=(: it != DEADLINE-EXCEEDED-ERROR):
      with-timeout TIMEOUT: interrupt.wait-for 1
    if timed-out:
      print "No interrupt for $TIMEOUT. Check that INT is wired to GPIO $INT-PIN."
      break
    // The pin is high, so the sample is ready. Reading it clears the pin.
    bytes := sensor.read-raw-data
    last-us = Time.monotonic-us
    if count == 0: first-us = last-us
    count++
    print "$(sensor.read-mag-compensated bytes) µT"

  if count > 1:
    seconds := (last-us - first-us) / 1_000_000.0
    rate := (count - 1) / seconds
    print "$count interrupts, about $(%.1f rate) Hz. Asked for $(%.1f Bmm350.odr-to-hz RATE) Hz."

  sensor.set-int-pin false
  interrupt.close
