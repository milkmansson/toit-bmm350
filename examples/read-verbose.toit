// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

import gpio
import i2c
import bmm350 show *
import math

READS := 10
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
  print "Taking raw data (once per loop), do $READS reads, and display compensated"
  print "and uncompensated reads, include magnitudes for both, and die temperature."
  sleep --ms=1000
  READS.repeat:
    raw := sensor.read-raw-data
    mag-raw := sensor.read-mag raw
    mag-raw-magnitude := magnitude mag-raw
    mag-c := sensor.read-mag-compensated raw
    mag-c-magnitude := magnitude mag-c
    sensortime := sensor.read-sensortime raw
    sensortime-string := "$("$sensortime".pad 9 --left '0')"
    temp-string := "$(%0.3f sensor.read-temp raw)"

    print "$("$it".pad 4 --left ' ') - $(sensor.read-mag raw) [mag: $(%0.3f mag-raw-magnitude)]    $(sensor.read-mag-compensated raw) [mag: $(%0.3f mag-c-magnitude)]  {$sensortime-string} $temp-string c"
    sleep --ms=100


magnitude point/math.Point3f -> float:
  return math.sqrt ( (point.x * point.x) + (point.y * point.y) + (point.z * point.z) )
