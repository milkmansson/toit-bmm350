// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import bmm350 show *
import math

READS := 100
SDA-PIN := 19
SCL-PIN := 20

main:
  print
  print
  bus := i2c.Bus
    --sda=gpio.Pin SDA-PIN
    --scl=gpio.Pin SCL-PIN
    --frequency=400_000

  if not (bus.test Bmm350.I2C-ADDRESS):
    print "Bus missing the device. stopping..."
    return

  device := bus.device Bmm350.I2C-ADDRESS
  sensor := Bmm350 device
  print "Hardware ID: 0x$(%02x sensor.get-chip-id)"
  print "Executing $READS (compensated) reads...."
  sleep --ms=1000
  READS.repeat:
    print "$("$it".pad 5 --left ' ')  -  $(sensor.read-mag-compensated) "
    sleep --ms=100
