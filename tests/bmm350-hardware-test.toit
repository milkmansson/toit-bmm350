// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

/**
Self-checking tests for a BMM350 wired to an ESP32. Needs real hardware.

Wire the module's SDA, SCL, power and ground to the ESP32, edit the pin numbers
  below if yours differ, and run with Jaguar:
```
jag run -d <your device> tests/bmm350-hardware-test.toit
```

Keep the board still and a few centimeters away from motors, magnets, speakers
  and steel tables while it runs. Every check prints PASS, FAIL, WARN or SKIP.
  A FAIL means something is wrong with the driver, the wiring or the chip. A
  WARN means a value is odd but not wrong, usually because of the surroundings.

The thresholds come from two places. Limits taken from the datasheet say so in
  their comments. The others are choices made for this test, and they are
  marked as such. Change them at the top of the file if your setup needs it.

Set $ROTATE-TEST to true to add a hands-on check. It asks you to turn the board
  through all orientations and checks that the field strength stays constant.
*/

import i2c
import log
import math show sqrt
import bmm350 show *
import .test-helpers

SDA-PIN ::= 21
SCL-PIN ::= 20
I2C-FREQUENCY ::= 400_000

/** Set to true to run the hands-on rotation check. */
ROTATE-TEST ::= false

/** How long the rotation check watches, in seconds. */
ROTATE-SECONDS ::= 20

// Datasheet limits. Temperature range: -40 °C to +85 °C. Field range: ±2000 µT.
TEMP-MIN-C ::= -40.0
TEMP-MAX-C ::= 85.0
FIELD-MAX-UT ::= 2000.0

// Datasheet, table 2: the output data rate is within typically ±2 % at room
// temperature. The test warns beyond that and fails at the wider limit, which
// is a choice made for this test.
RATE-WARN-RATIO ::= 0.02
RATE-FAIL-RATIO ::= 0.05

// Datasheet, table 2: noise at 100 Hz with 2 samples averaged is 190 nT rms on
// X and Y and 450 nT rms on Z. The test warns at 1.5 times that and fails at 3
// times, which are choices made for this test.
NOISE-XY-UT ::= 0.19
NOISE-Z-UT ::= 0.45
NOISE-WARN-FACTOR ::= 1.5
NOISE-FAIL-FACTOR ::= 3.0

// Choices made for this test. Earth's field is commonly given as roughly 25 to
// 65 µT, and the room adds to it, so the test only warns outside 15 to 100 µT.
// Room temperature gives a die temperature near 20 to 40 °C.
FIELD-WARN-LOW-UT ::= 15.0
FIELD-WARN-HIGH-UT ::= 100.0
TEMP-WARN-LOW-C ::= 10.0
TEMP-WARN-HIGH-C ::= 50.0

// Choices made for this test: how far the field may move across a magnetic
// reset, a soft reset and a forced measurement on a still board.
SHIFT-WARN-UT ::= 2.0
SHIFT-FAIL-UT ::= 10.0

// Choice made for this test: how much the field strength may vary when the
// board is rotated in a roughly uniform field.
ROTATE-MAX-SPREAD ::= 0.25

RATES ::= [
  Bmm350.ODR-400HZ,
  Bmm350.ODR-200HZ,
  Bmm350.ODR-100HZ,
  Bmm350.ODR-50HZ,
  Bmm350.ODR-25HZ,
  Bmm350.ODR-12-5HZ,
]

class Stats:
  count/int := 0
  sum/float := 0.0
  sum-of-squares/float := 0.0
  low/float := 1.0e30
  high/float := -1.0e30

  add value/float -> none:
    count++
    sum += value
    sum-of-squares += value * value
    if value < low: low = value
    if value > high: high = value

  mean -> float:
    return sum / count

  std-dev -> float:
    variance := sum-of-squares / count - mean * mean
    return variance > 0.0 ? sqrt variance : 0.0

magnitude x/float y/float z/float -> float:
  return sqrt (x * x + y * y + z * z)

// Averages some compensated readings, to compare before and after an action.
average-field sensor/Bmm350 samples/int=10 -> List:
  x := Stats
  y := Stats
  z := Stats
  samples.repeat:
    field := sensor.read-mag-compensated
    x.add field.x
    y.add field.y
    z.add field.z
  return [x.mean, y.mean, z.mean]

distance a/List b/List -> float:
  return magnitude (a[0] - b[0]) (a[1] - b[1]) (a[2] - b[2])

// Judges a shift in the field after some action, as a warning or a failure.
judge-shift run/TestRun name/string before/List after/List -> none:
  shift := distance before after
  if shift > SHIFT-FAIL-UT:
    throw "field moved by $(%.2f shift) µT"
  if shift > SHIFT-WARN-UT:
    run.warn name "field moved by $(%.2f shift) µT"

main:
  run := TestRun
  run.section "Bus and identity"

  bus/i2c.Bus? := null
  sensor/Bmm350? := null

  run.check "I2C bus opens":
    bus = i2c.Bus
        --sda=SDA-PIN
        --scl=SCL-PIN
        --frequency=I2C-FREQUENCY

  run.check "chip answers at 0x$(%02x Bmm350.I2C-ADDRESS)":
    if not bus: throw "no bus"
    if not (bus.test Bmm350.I2C-ADDRESS):
      throw "nothing at 0x$(%02x Bmm350.I2C-ADDRESS). Check wiring and the ADSEL pin (0x$(%02x Bmm350.I2C-ADDRESS-ALT) when high)."

  run.check "driver starts":
    sensor = Bmm350 (bus.device Bmm350.I2C-ADDRESS) --logger=(log.default.with-level log.WARN-LEVEL)
    require-equals 0b0011 sensor.get-chip-id

  if sensor == null:
    run.skip "everything else" "the driver did not start"
    run.finish
    return

  run-hardware-tests sensor run
  run.finish

/**
Runs every check that only needs a started driver.

Separate from $main so that the checks can also run against a simulated chip.
*/
run-hardware-tests sensor/Bmm350 run/TestRun -> none:
  run.section "Readings"

  run.check "temperature is inside the chip's range":
    temp := sensor.read-temp
    print "        die temperature $(%.2f temp) °C"
    if temp < TEMP-MIN-C or temp > TEMP-MAX-C:
      throw "$(%.2f temp) °C is outside the datasheet range of $TEMP-MIN-C to $TEMP-MAX-C °C"
    if temp < TEMP-WARN-LOW-C or temp > TEMP-WARN-HIGH-C:
      run.warn "temperature" "$(%.2f temp) °C is unusual for a room"

  run.check "field reads sensibly":
    bytes := sensor.read-raw-data
    plain := sensor.read-mag bytes
    comp := sensor.read-mag-compensated bytes
    strength := magnitude comp.x comp.y comp.z
    print "        uncompensated $plain"
    print "        compensated   $comp  ($(%.2f strength) µT)"
    if strength == 0.0: throw "the field reads exactly zero on every axis"
    if strength != strength: throw "the field is not a number"
    if strength > FIELD-MAX-UT: throw "$(%.1f strength) µT is beyond the ±$(FIELD-MAX-UT.to-int) µT range"
    if strength < FIELD-WARN-LOW-UT or strength > FIELD-WARN-HIGH-UT:
      run.warn "field strength" "$(%.1f strength) µT. Earth's field is roughly 25 to 65 µT, so check for magnets or steel nearby."

  run.check "one burst gives matching values":
    bytes := sensor.read-raw-data
    first := sensor.read-mag-compensated bytes
    second := sensor.read-mag-compensated bytes
    require-equals first.x second.x
    require-equals first.y second.y
    require-equals first.z second.z

  run.section "Configuration"

  run.check "every rate and averaging pair is accepted":
    mismatches := []
    9.repeat: | index |
      odr := Bmm350.ODR-400HZ + index
      4.repeat: | avg |
        sensor.configure-mag --odr=odr --avg=avg
        expected := min avg (Bmm350.max-avg-for-odr odr)
        got-odr := sensor.get-odr
        got-avg := sensor.get-avg
        if got-odr != odr or got-avg != expected:
          mismatches.add "asked odr $odr avg $avg, expected odr $odr avg $expected, chip reports odr $got-odr avg $got-avg"
    if not mismatches.is-empty:
      throw "$mismatches.size pairs differ. First: $mismatches[0]"
    require-not sensor.is-pmu-error --message="the chip flagged a power command error"

  run.section "Timing"

  RATES.do: | odr |
    name := "sensortime ticks match $(Bmm350.odr-to-hz odr) Hz"
    run.check name:
      sensor.configure-mag --odr=odr --avg=Bmm350.AVG-NONE
      3.repeat: sensor.read-raw-data
      expected := 1_000_000.0 / (Bmm350.odr-to-hz odr) / Bmm350.SENSORTIME-TICK-US
      deltas := []
      previous := sensor.read-sensortime sensor.read-raw-data
      20.repeat:
        now := sensor.read-sensortime sensor.read-raw-data
        deltas.add (Bmm350.sensortime-elapsed previous now)
        previous = now
      deltas.sort --in-place
      median := deltas[deltas.size / 2]
      // The datasheet says sensortime and ODR share one clock (section 5.2.2),
      // so the median should be exact. The allowance is a choice made for this test.
      allowance := max 2.0 (expected * 0.01)
      require-close expected median allowance "median ticks between samples"
      missed := 0
      deltas.do: if it > expected * 1.5: missed++
      if missed > 0:
        run.warn name "$missed of 20 intervals were longer than one sample. The host missed samples."

  run.check "sensor clock agrees with the ESP32 clock":
    sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-NONE
    3.repeat: sensor.read-raw-data
    first-ticks := sensor.read-sensortime sensor.read-raw-data
    first-us := Time.monotonic-us
    while Time.monotonic-us - first-us < 3_000_000: sensor.read-raw-data
    last-ticks := sensor.read-sensortime sensor.read-raw-data
    last-us := Time.monotonic-us
    chip-us := Bmm350.sensortime-to-us (Bmm350.sensortime-elapsed first-ticks last-ticks)
    ratio := chip-us / (last-us - first-us)
    print "        chip time / ESP32 time = $(%.4f ratio)"
    error := (ratio - 1.0).abs
    if error > RATE-FAIL-RATIO: throw "the chip clock is off by $(%.1f error * 100) %"
    if error > RATE-WARN-RATIO:
      run.warn "clock ratio" "off by $(%.1f error * 100) %. The datasheet says typically within 2 %."

  run.section "Noise on a still board"

  run.check "noise at 100 Hz, 2 samples averaged":
    sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-2-SAMPLES
    3.repeat: sensor.read-raw-data
    x := Stats
    y := Stats
    z := Stats
    200.repeat:
      field := sensor.read-mag-compensated
      x.add field.x
      y.add field.y
      z.add field.z
    print "        std dev x $(%.3f x.std-dev), y $(%.3f y.std-dev), z $(%.3f z.std-dev) µT"
    print "        datasheet x/y $(%.2f NOISE-XY-UT), z $(%.2f NOISE-Z-UT) µT rms"
    if x.std-dev > NOISE-XY-UT * NOISE-FAIL-FACTOR or y.std-dev > NOISE-XY-UT * NOISE-FAIL-FACTOR or z.std-dev > NOISE-Z-UT * NOISE-FAIL-FACTOR:
      throw "noise is more than $NOISE-FAIL-FACTOR times the datasheet figure. Check the supply, the wiring and nearby motors."
    if x.std-dev > NOISE-XY-UT * NOISE-WARN-FACTOR or y.std-dev > NOISE-XY-UT * NOISE-WARN-FACTOR or z.std-dev > NOISE-Z-UT * NOISE-WARN-FACTOR:
      run.warn "noise" "above $NOISE-WARN-FACTOR times the datasheet figure. Something nearby may be moving or switching."

  run.section "Modes and resets"

  run.check "forced mode gives one measurement, then the chip suspends":
    sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-NONE
    before := average-field sensor
    sensor.set-mode Bmm350.MODE-SUSPEND
    sensor.set-mode Bmm350.MODE-FORCED
    comp := sensor.read-mag-compensated
    judge-shift run "forced mode" before [comp.x, comp.y, comp.z]
    // Once suspended, no further data arrives.
    require-throw "BMM350 data ready timeout (is it in suspend mode?)": sensor.read-raw-data
    sensor.set-mode Bmm350.MODE-NORMAL

  run.check "one axis can be switched off and on":
    sensor.enable-axis --z=false
    bytes := sensor.read-raw-data
    require-equals 0.0 (sensor.read-mag bytes).z
    require-equals 0.0 (sensor.read-mag-compensated bytes).z
    require (sensor.read-mag-compensated bytes).x != 0.0
    sensor.enable-axis
    require (sensor.read-mag-compensated).z != 0.0

  run.check "magnetic reset keeps the reading":
    before := average-field sensor
    sensor.magnetic-reset
    after := average-field sensor
    judge-shift run "magnetic reset" before after

  run.check "soft reset and setup again keep the reading":
    before := average-field sensor
    sensor.soft-reset
    sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-NONE
    sensor.set-data-ready-check true
    sensor.set-mode Bmm350.MODE-NORMAL
    after := average-field sensor
    judge-shift run "soft reset" before after

  if ROTATE-TEST:
    run.section "Rotation"
    run.check "field strength stays constant while the board turns":
      check-rotation sensor run
  else:
    run.skip "rotation" "set ROTATE-TEST to true to run it"

// Watches the field while the user turns the board, and compares how constant
// the strength stays with and without the OTP compensation.
check-rotation sensor/Bmm350 run/TestRun -> none:
  sensor.configure-mag --odr=Bmm350.ODR-50HZ --avg=Bmm350.AVG-8-SAMPLES
  print "        Turn the board slowly through every orientation for $ROTATE-SECONDS s."
  print "        Starting in 5 s."
  sleep --ms=5000
  print "        Go."

  plain-strength := Stats
  comp-strength := Stats
  axes := [Stats, Stats, Stats]
  start := Time.monotonic-us
  while Time.monotonic-us - start < ROTATE-SECONDS * 1_000_000:
    bytes := sensor.read-raw-data
    plain := sensor.read-mag bytes
    comp := sensor.read-mag-compensated bytes
    plain-strength.add (magnitude plain.x plain.y plain.z)
    comp-strength.add (magnitude comp.x comp.y comp.z)
    axes[0].add comp.x
    axes[1].add comp.y
    axes[2].add comp.z
  print "        Done."

  plain-spread := (plain-strength.high - plain-strength.low) / plain-strength.mean
  comp-spread := (comp-strength.high - comp-strength.low) / comp-strength.mean
  print "        strength, uncompensated: mean $(%.1f plain-strength.mean) µT, spread $(%.1f plain-spread * 100) %"
  print "        strength, compensated:   mean $(%.1f comp-strength.mean) µT, spread $(%.1f comp-spread * 100) %"
  print "        hard-iron centre (middle of each axis range): $(%.1f (axes[0].low + axes[0].high) / 2), $(%.1f (axes[1].low + axes[1].high) / 2), $(%.1f (axes[2].low + axes[2].high) / 2) µT"
  if axes[0].high - axes[0].low < 10.0 and axes[1].high - axes[1].low < 10.0 and axes[2].high - axes[2].low < 10.0:
    throw "the field hardly changed, so the board was probably not turned"
  if comp-spread > ROTATE-MAX-SPREAD:
    throw "strength varies by $(%.1f comp-spread * 100) % with orientation. A steel object or magnet is probably nearby."
  if comp-spread > plain-spread + 0.02:
    run.warn "rotation" "compensation made the strength less constant ($(%.1f comp-spread * 100) % against $(%.1f plain-spread * 100) %)."
