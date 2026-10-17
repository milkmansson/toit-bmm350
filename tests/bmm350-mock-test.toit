// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by a Zero-Clause BSD license that can
// be found in the EXAMPLES_LICENSE file.

/**
Tests the BMM350 driver against a simulated chip. No hardware is needed.

Run on the host with:
```
toit pkg install
toit tests/bmm350-mock-test.toit
```

The compensation checks compare the driver's output with numbers produced by
  Bosch Sensortec's own C code (see golden/bmm350-golden.c). The register
  checks use $MockBmm350Registers, which follows the datasheet closely enough
  to catch the mistakes that matter: missed dummy bytes, clobbered neighbouring
  bits, illegal power commands, and updates sent too close together.
*/

import log
import bmm350 show *
import .golden-vectors
import .mock-bmm350
import bmm350.calibration show MagCalibration
import .test-helpers

REG-AGGR-SET ::= 0x04
REG-AXIS-SET ::= 0x05
REG-PMU-CMD ::= 0x06
REG-INT-CTRL ::= 0x2E
REG-OTP-CMD ::= 0x50

CMD-SUSPEND ::= 0
CMD-NORMAL ::= 1
CMD-UPDATE ::= 2
CMD-FORCED ::= 3
CMD-FGR ::= 5
CMD-BR ::= 7

// Tolerances for the golden vectors. Bosch computes in 32-bit floats.
MAG-TOLERANCE ::= 0.005
TEMP-TOLERANCE ::= 0.001

new-sensor mock/MockBmm350Device -> Bmm350
    --otp/List?=null
    --odr/int=Bmm350.ODR-400HZ
    --avg/int=Bmm350.AVG-NONE
    --calibration/MagCalibration?=null:
  if otp: otp.size.repeat: mock.registers.otp-words[it] = otp[it]
  logger := log.default.with-level log.ERROR-LEVEL
  return Bmm350 mock --odr=odr --avg=avg --calibration=calibration --logger=logger

// Puts the chip in suspend mode, so that no samples arrive on their own.
suspended mock/MockBmm350Device -> Bmm350:
  sensor := new-sensor mock
  sensor.set-mode Bmm350.MODE-SUSPEND
  return sensor

// Returns the index of the first write of `[register, value]` at or after
// `from`, or -1.
index-of-write mock/MockBmm350Device register/int value/int --from/int=0 -> int:
  writes := mock.registers.writes
  for i := from; i < writes.size; i++:
    if writes[i][0] == register and writes[i][1] == value: return i
  return -1

count-writes mock/MockBmm350Device register/int value/int -> int:
  count := 0
  mock.registers.writes.do: | write |
    if write[0] == register and write[1] == value: count++
  return count

main:
  run := TestRun

  run.section "Boot"

  run.check "boots and identifies the chip":
    mock := MockBmm350Device
    sensor := new-sensor mock
    require-equals 0b0011 sensor.get-chip-id
    require mock.registers.is-normal

  run.check "ignores the OTP revision in the chip ID":
    mock := MockBmm350Device
    mock.registers.chip-id = 0x37
    sensor := new-sensor mock
    require-equals 0b0011 sensor.get-chip-id

  run.check "rejects a chip with the wrong ID":
    mock := MockBmm350Device
    mock.registers.chip-id = 0x44
    require-throw "Unexpected chip id [4]": new-sensor mock

  run.check "reports an OTP read error":
    mock := MockBmm350Device
    mock.registers.otp-fail = true
    require-throw "BMM350 OTP error: Page Read Error": new-sensor mock

  run.check "boot order: OTP off, then bit reset, flux guide reset, normal":
    mock := MockBmm350Device
    new-sensor mock
    otp-off := index-of-write mock REG-OTP-CMD 0x80
    bit-reset := index-of-write mock REG-PMU-CMD CMD-BR
    flux-reset := index-of-write mock REG-PMU-CMD CMD-FGR
    normal := index-of-write mock REG-PMU-CMD CMD-NORMAL
    require otp-off >= 0
    require otp-off < bit-reset
    require bit-reset < flux-reset
    require flux-reset < normal

  run.check "boot sends no illegal power commands":
    mock := MockBmm350Device
    new-sensor mock
    require-equals 0 mock.registers.illegal-count

  run.check "boot with odr and avg arguments":
    mock := MockBmm350Device
    sensor := new-sensor mock --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-8-SAMPLES
    require-equals Bmm350.ODR-100HZ sensor.get-odr
    require-equals Bmm350.AVG-4-SAMPLES sensor.get-avg

  run.section "Compensation against Bosch reference values"

  run.check "uncompensated field":
    GOLDEN-CASES.do: | case |
      sensor := new-sensor MockBmm350Device --otp=case["otp"]
      mag := sensor.read-mag case["raw"]
      expected := case["mag"]
      require-close expected[0] mag.x MAG-TOLERANCE "$case["name"] x"
      require-close expected[1] mag.y MAG-TOLERANCE "$case["name"] y"
      require-close expected[2] mag.z MAG-TOLERANCE "$case["name"] z"

  run.check "compensated field":
    GOLDEN-CASES.do: | case |
      sensor := new-sensor MockBmm350Device --otp=case["otp"]
      mag := sensor.read-mag-compensated case["raw"]
      expected := case["mag-comp"]
      require-close expected[0] mag.x MAG-TOLERANCE "$case["name"] x"
      require-close expected[1] mag.y MAG-TOLERANCE "$case["name"] y"
      require-close expected[2] mag.z MAG-TOLERANCE "$case["name"] z"

  run.check "temperature with the OTP trim":
    GOLDEN-CASES.do: | case |
      sensor := new-sensor MockBmm350Device --otp=case["otp"]
      temp := sensor.read-temp case["raw"]
      require-close case["temp-comp"] temp TEMP-TOLERANCE case["name"]

  run.check "trim values decoded from OTP words":
    GOLDEN-CASES.do: | case |
      trim := Bmm350Trim
      case["otp"].size.repeat: trim[it] = case["otp"][it]
      require-equals case["offsets"][0] trim.offset-x
      require-equals case["offsets"][1] trim.offset-y
      require-equals case["offsets"][2] trim.offset-z
      require-close case["t0"] trim.temp-reference 0.0001 "$case["name"] reference temperature"

  run.check "21-bit values read the same sign-extended or zero-padded":
    sensor := new-sensor MockBmm350Device
    extended := sensor.read-mag #[0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x10, 0x01, 0x00, 0x00, 0, 0, 0, 0, 0, 0]
    padded := sensor.read-mag #[0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x10, 0x01, 0x00, 0x00, 0, 0, 0, 0, 0, 0]
    require-equals extended.x padded.x
    require extended.x < 0.0
    require extended.y < 0.0  // 0x100000 is the most negative 21-bit value.

  run.section "Registers"

  run.check "skips the dummy bytes on a burst read":
    mock := MockBmm350Device
    sensor := suspended mock
    bytes := ByteArray 15: 0x10 + it
    mock.registers.inject-bytes bytes
    require-bytes-equal bytes sensor.read-raw-data

  run.check "masked writes keep the neighbouring bits":
    mock := MockBmm350Device
    sensor := suspended mock
    mock.registers.poke REG-INT-CTRL 0x0A
    sensor.set-data-ready-check true
    require-equals 0x8A (mock.registers.peek REG-INT-CTRL)
    sensor.set-data-ready-check false
    require-equals 0x0A (mock.registers.peek REG-INT-CTRL)
    sensor.set-int-pin-polarity false
    require-equals 0x08 (mock.registers.peek REG-INT-CTRL)
    sensor.set-int-pin false
    require-equals 0x00 (mock.registers.peek REG-INT-CTRL)

  run.check "latched mode writes its own bit":
    mock := MockBmm350Device
    sensor := suspended mock
    mock.registers.poke REG-INT-CTRL 0x80
    sensor.set-int-latched-mode true
    require-equals 0x81 (mock.registers.peek REG-INT-CTRL)
    sensor.set-int-latched-mode false
    require-equals 0x80 (mock.registers.peek REG-INT-CTRL)

  run.check "pin drive and polarity write their own bits":
    mock := MockBmm350Device
    sensor := suspended mock
    mock.registers.poke REG-INT-CTRL 0x00
    sensor.set-int-pin-push-pull true
    require-equals 0x04 (mock.registers.peek REG-INT-CTRL)
    sensor.set-int-pin-polarity true
    require-equals 0x06 (mock.registers.peek REG-INT-CTRL)
    sensor.set-int-pin true
    require-equals 0x0E (mock.registers.peek REG-INT-CTRL)
    require sensor.is-int-pin-set

  run.section "Data ready and sensortime"

  run.check "data ready looks only at its own bit":
    mock := MockBmm350Device
    sensor := suspended mock
    mock.registers.int-status-noise = 0x01
    require-not sensor.is-data-ready

  run.check "leaving normal mode clears a leftover data ready flag":
    mock := MockBmm350Device
    sensor := new-sensor mock
    sleep --ms=20  // Let a sample come in that nobody reads.
    sensor.set-mode Bmm350.MODE-SUSPEND
    require-not sensor.is-data-ready

  run.check "data ready clears when read":
    mock := MockBmm350Device
    sensor := suspended mock
    mock.registers.inject-sample
    require sensor.is-data-ready
    require-not sensor.is-data-ready

  run.check "read-raw-data times out in suspend mode":
    mock := MockBmm350Device
    sensor := suspended mock
    require-throw "BMM350 data ready timeout (is it in suspend mode?)": sensor.read-raw-data

  run.check "read-raw-data does not wait when data ready is off":
    mock := MockBmm350Device
    sensor := suspended mock
    sensor.set-data-ready-check false
    start := Time.monotonic-us
    sensor.read-raw-data
    require (Time.monotonic-us - start) < 200_000

  run.check "sensortime is unsigned":
    sensor := new-sensor MockBmm350Device
    bytes := ByteArray 15
    bytes[14] = 0x90
    require-equals 0x900000 (sensor.read-sensortime bytes)
    bytes[12] = 0xFF
    bytes[13] = 0xFF
    bytes[14] = 0xFF
    require-equals 0xFFFFFF (sensor.read-sensortime bytes)

  run.check "sensortime arithmetic":
    require-equals 0x20 (Bmm350.sensortime-elapsed 0xFFFFF0 0x10)
    require-equals 256 (Bmm350.sensortime-elapsed 1000 1256)
    require-equals 10000.0 (Bmm350.sensortime-to-us 256)

  run.check "output data rates in Hz":
    require-equals 400.0 (Bmm350.odr-to-hz Bmm350.ODR-400HZ)
    require-equals 100.0 (Bmm350.odr-to-hz Bmm350.ODR-100HZ)
    require-equals 12.5 (Bmm350.odr-to-hz Bmm350.ODR-12-5HZ)
    require-equals 1.5625 (Bmm350.odr-to-hz Bmm350.ODR-1-5625HZ)

  run.section "Rate and averaging"

  run.check "highest averaging per rate follows datasheet table 5":
    require-equals Bmm350.AVG-NONE (Bmm350.max-avg-for-odr Bmm350.ODR-400HZ)
    require-equals Bmm350.AVG-2-SAMPLES (Bmm350.max-avg-for-odr Bmm350.ODR-200HZ)
    require-equals Bmm350.AVG-4-SAMPLES (Bmm350.max-avg-for-odr Bmm350.ODR-100HZ)
    require-equals Bmm350.AVG-8-SAMPLES (Bmm350.max-avg-for-odr Bmm350.ODR-50HZ)
    require-equals Bmm350.AVG-8-SAMPLES (Bmm350.max-avg-for-odr Bmm350.ODR-1-5625HZ)

  run.check "every rate and averaging pair ends up allowed":
    mock := MockBmm350Device
    sensor := suspended mock
    for odr := Bmm350.ODR-400HZ; odr <= Bmm350.ODR-1-5625HZ; odr++:
      for avg := Bmm350.AVG-NONE; avg <= Bmm350.AVG-8-SAMPLES; avg++:
        sensor.configure-mag --odr=odr --avg=avg
        expected-avg := min avg (Bmm350.max-avg-for-odr odr)
        require-equals odr sensor.get-odr
        require-equals expected-avg sensor.get-avg

  run.check "never writes a pair the chip rejects":
    mock := MockBmm350Device
    sensor := new-sensor mock --odr=Bmm350.ODR-400HZ --avg=Bmm350.AVG-8-SAMPLES
    sensor.set-mode Bmm350.MODE-SUSPEND
    sensor.configure-mag --odr=Bmm350.ODR-200HZ --avg=Bmm350.AVG-8-SAMPLES
    sensor.set-avg Bmm350.AVG-8-SAMPLES
    sensor.set-odr Bmm350.ODR-100HZ
    mock.registers.writes.do: | write |
      if write[0] == REG-AGGR-SET:
        odr := write[1] & 0x0F
        avg := (write[1] >> 4) & 0x03
        require avg <= (Bmm350.max-avg-for-odr odr)

  run.check "one update command per configuration":
    mock := MockBmm350Device
    sensor := suspended mock
    before := count-writes mock REG-PMU-CMD CMD-UPDATE
    sensor.configure-mag --odr=Bmm350.ODR-50HZ --avg=Bmm350.AVG-8-SAMPLES
    require-equals (before + 1) (count-writes mock REG-PMU-CMD CMD-UPDATE)

  run.check "set-odr and set-avg keep the other setting":
    mock := MockBmm350Device
    sensor := suspended mock
    sensor.configure-mag --odr=Bmm350.ODR-25HZ --avg=Bmm350.AVG-4-SAMPLES
    sensor.set-odr Bmm350.ODR-12-5HZ
    require-equals Bmm350.AVG-4-SAMPLES sensor.get-avg
    sensor.set-avg Bmm350.AVG-8-SAMPLES
    require-equals Bmm350.ODR-12-5HZ sensor.get-odr
    sensor.set-odr Bmm350.ODR-400HZ
    require-equals Bmm350.AVG-NONE sensor.get-avg
    sensor.set-avg Bmm350.AVG-8-SAMPLES
    require-equals Bmm350.AVG-NONE sensor.get-avg

  run.check "rejects values that are not rates or averaging settings":
    sensor := suspended MockBmm350Device
    require-throw "Invalid output data rate 1": sensor.configure-mag --odr=1
    require-throw "Invalid output data rate 11": sensor.configure-mag --odr=11
    require-throw "Invalid averaging 4": sensor.configure-mag --avg=4

  run.check "samples arrive at the configured 100 Hz":
    mock := MockBmm350Device
    sensor := new-sensor mock --odr=Bmm350.ODR-100HZ
    previous := sensor.read-sensortime sensor.read-raw-data
    off := 0
    20.repeat:
      now := sensor.read-sensortime sensor.read-raw-data
      if (Bmm350.sensortime-elapsed previous now) != 256: off++
      previous = now
    require off <= 2

  run.check "samples arrive at the configured 400 Hz":
    mock := MockBmm350Device
    sensor := new-sensor mock --odr=Bmm350.ODR-400HZ
    previous := sensor.read-sensortime sensor.read-raw-data
    off := 0
    40.repeat:
      now := sensor.read-sensortime sensor.read-raw-data
      if (Bmm350.sensortime-elapsed previous now) != 64: off++
      previous = now
    require off <= 4

  run.section "Power modes"

  run.check "forced mode goes through suspend":
    mock := MockBmm350Device
    sensor := new-sensor mock
    sensor.set-mode Bmm350.MODE-FORCED
    suspend := index-of-write mock REG-PMU-CMD CMD-SUSPEND
    forced := index-of-write mock REG-PMU-CMD CMD-FORCED
    require suspend >= 0
    require suspend < forced
    require-equals 0 mock.registers.illegal-count
    require-not mock.registers.is-normal
    require-equals 15 (sensor.read-raw-data).size

  run.check "normal mode from normal mode is accepted":
    mock := MockBmm350Device
    sensor := new-sensor mock
    sensor.set-mode Bmm350.MODE-NORMAL
    require mock.registers.is-normal
    require-equals 0 mock.registers.illegal-count

  run.check "rejects power modes the driver does not offer":
    sensor := new-sensor MockBmm350Device
    require-throw "Unsupported power mode 2": sensor.set-mode 2
    require-throw "Unsupported power mode 7": sensor.set-mode 7

  run.check "changing rate while running sends no illegal command":
    mock := MockBmm350Device
    sensor := new-sensor mock
    sensor.configure-mag --odr=Bmm350.ODR-200HZ --avg=Bmm350.AVG-2-SAMPLES
    sensor.set-odr Bmm350.ODR-100HZ
    require-equals 0 mock.registers.illegal-count
    require mock.registers.is-normal
    require-equals Bmm350.ODR-100HZ sensor.get-odr

  run.check "magnetic reset from normal mode":
    mock := MockBmm350Device
    sensor := new-sensor mock
    before := mock.registers.writes.size
    sensor.magnetic-reset
    suspend := index-of-write mock REG-PMU-CMD CMD-SUSPEND --from=before
    bit-reset := index-of-write mock REG-PMU-CMD CMD-BR --from=before
    flux-reset := index-of-write mock REG-PMU-CMD CMD-FGR --from=before
    normal := index-of-write mock REG-PMU-CMD CMD-NORMAL --from=before
    require suspend >= 0
    require suspend < bit-reset
    require bit-reset < flux-reset
    require flux-reset < normal
    require-equals 0 mock.registers.illegal-count
    require mock.registers.is-normal

  run.check "soft reset leaves a chip that can be set up again":
    mock := MockBmm350Device
    sensor := new-sensor mock --odr=Bmm350.ODR-50HZ
    sensor.soft-reset
    require-not mock.registers.is-normal
    require-equals 0x14 (mock.registers.peek REG-AGGR-SET)
    sensor.configure-mag --odr=Bmm350.ODR-100HZ --avg=Bmm350.AVG-4-SAMPLES
    sensor.set-data-ready-check true
    sensor.set-mode Bmm350.MODE-NORMAL
    require-equals 15 (sensor.read-raw-data).size
    require-equals 0 mock.registers.illegal-count

  run.section "Axes"

  run.check "disabled axis reads zero and the chip returns to normal mode":
    mock := MockBmm350Device
    sensor := new-sensor mock
    mock.registers.raw-x = 1000
    mock.registers.raw-y = 2000
    mock.registers.raw-z = 3000
    sensor.enable-axis --z=false
    require-equals 0x03 (mock.registers.peek REG-AXIS-SET)
    require mock.registers.is-normal
    require-equals 0 mock.registers.illegal-count
    bytes := sensor.read-raw-data
    require (sensor.read-mag bytes).x != 0.0
    require-equals 0.0 (sensor.read-mag bytes).z
    require-equals 0.0 (sensor.read-mag-compensated bytes).z
    sensor.enable-axis
    require-equals 0x07 (mock.registers.peek REG-AXIS-SET)

  run.check "refuses to disable every axis":
    sensor := new-sensor MockBmm350Device
    require-throw "At least one axis must stay enabled": sensor.enable-axis --x=false --y=false --z=false

  run.section "Calibration"

  calibration := MagCalibration
      --offset-x=3.0
      --offset-y=-2.0
      --offset-z=1.0
      --scale-x=1.1
      --scale-y=0.9
      --scale-z=1.2
  golden := GOLDEN-CASES[0]

  run.check "the constructor argument corrects compensated readings only":
    plain := new-sensor MockBmm350Device --otp=golden["otp"]
    corrected := new-sensor MockBmm350Device --otp=golden["otp"] --calibration=calibration
    bytes := golden["raw"]
    expected := calibration.apply (plain.read-mag-compensated bytes)
    got := corrected.read-mag-compensated bytes
    require-close expected.x got.x 1.0e-9 "x"
    require-close expected.y got.y 1.0e-9 "y"
    require-close expected.z got.z 1.0e-9 "z"
    require-equals (plain.read-mag bytes).x (corrected.read-mag bytes).x
    require-equals (plain.read-mag bytes).y (corrected.read-mag bytes).y
    require-equals (plain.read-mag bytes).z (corrected.read-mag bytes).z

  run.check "a calibration can be set and removed later":
    sensor := new-sensor MockBmm350Device --otp=golden["otp"]
    bytes := golden["raw"]
    before := sensor.read-mag-compensated bytes
    sensor.calibration = calibration
    expected := calibration.apply before
    got := sensor.read-mag-compensated bytes
    require-close expected.x got.x 1.0e-9 "x"
    require-close expected.y got.y 1.0e-9 "y"
    require-close expected.z got.z 1.0e-9 "z"
    sensor.calibration = null
    after := sensor.read-mag-compensated bytes
    require-equals before.x after.x
    require-equals before.y after.y
    require-equals before.z after.z

  run.check "a disabled axis still reads zero with a calibration":
    mock := MockBmm350Device
    sensor := new-sensor mock --calibration=calibration
    mock.registers.raw-x = 1000
    mock.registers.raw-y = 2000
    mock.registers.raw-z = 3000
    sensor.enable-axis --z=false
    bytes := sensor.read-raw-data
    require-equals 0.0 (sensor.read-mag-compensated bytes).z
    require (sensor.read-mag-compensated bytes).x != 0.0

  run.finish
