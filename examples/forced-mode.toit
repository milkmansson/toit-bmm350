// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

// Takes one reading a second in forced mode. Between readings the chip sits in
// suspend mode, which suits battery projects that only need an occasional
// reading.

import gpio
import i2c
import bmm350 show Bmm350

SDA-PIN := 21
SCL-PIN := 20

main:
  bus := i2c.Bus
    --sda=SDA-PIN
    --scl=SCL-PIN
    --frequency=400_000

  if not (bus.test Bmm350.I2C-ADDRESS):
    print "No BMM350 found on the bus. Check the wiring and the I2C address."
    return

  sensor := Bmm350 (bus.device Bmm350.I2C-ADDRESS)

  10.repeat:
    // The chip only runs forced measurements from suspend mode. The driver
    // suspends it first if it is still in normal mode.
    sensor.set-mode Bmm350.MODE-FORCED
    // Read the bytes once and convert them twice. The chip is back in suspend
    // mode by now, so a second read of the data would wait in vain.
    bytes := sensor.read-raw-data
    print "$(sensor.read-mag-compensated bytes) µT  $(%.1f sensor.read-temp bytes) °C"
    sleep --ms=1000
