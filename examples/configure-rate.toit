// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

// Picks an output data rate and an averaging setting, then reads back what the
// chip uses. Averaging lowers noise at the cost of current.

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

  // The chip allows 8 averaged samples at 50 Hz but only 4 at 100 Hz. When you
  // ask for more than a rate allows, the driver uses the most the rate allows.
  sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-8-SAMPLES
  print "Asked for 100 Hz with 8 samples averaged."
  print "The chip now runs at $(Bmm350.odr-to-hz sensor.get-odr) Hz with $(1 << sensor.get-avg) samples averaged."

  10.repeat:
    print "$sensor.read-mag-compensated µT"
