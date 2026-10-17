// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.

import io
import serial
import log
import math show Point3f

/**
Toit driver for the Bosch Sensortec BMM350 magnetometer.

The BMM350 measures the magnetic field on three axes and reports it in
  microtesla (µT), along with the die temperature in °C. This driver talks to
  the chip over I2C. The chip also supports I3C, which this driver does not
  implement.

The constructor boots the chip, loads its factory trim values, and starts
  continuous measurement in normal mode at 400 Hz without averaging. After
  that, $read-mag-compensated returns a temperature-corrected field reading:

```
import gpio
import i2c
import bmm350

main:
  bus := i2c.Bus --sda=(gpio.Pin 19) --scl=(gpio.Pin 20)
  sensor := bmm350.Bmm350 (bus.device bmm350.Bmm350.I2C-ADDRESS)
  print sensor.read-mag-compensated
```

# Reading data
Each call to $read-raw-data returns one burst of bytes that holds a single
  measurement: three axes, temperature and sensortime. The $read-mag,
  $read-mag-compensated, $read-temp and $read-sensortime methods all accept
  those bytes. Passing the same bytes to several of them keeps every value
  from the same instant. Without an argument, each call takes a fresh
  measurement.

# Inheritance
The BMM350 returns two dummy bytes at the start of every register read, so an
  n-byte read needs n+2 bytes on the bus, and the first two are discarded
  (datasheet section 9.2.3). $read-register_ and $write-register_ take care of
  that, including the read half of a masked write.
*/
class Bmm350:
  /** I2C address when the ADSEL pin is low. */
  static I2C-ADDRESS      ::= 0x14

  /** I2C address when the ADSEL pin is high. */
  static I2C-ADDRESS-ALT  ::= 0x15

  /** Output data rate of 400 Hz. Limits averaging to $AVG-NONE. */
  static ODR-400HZ    ::= 0x02
  /** Output data rate of 200 Hz. Allows averaging up to $AVG-2-SAMPLES. */
  static ODR-200HZ    ::= 0x03
  /** Output data rate of 100 Hz. Allows averaging up to $AVG-4-SAMPLES. */
  static ODR-100HZ    ::= 0x04
  /** Output data rate of 50 Hz. */
  static ODR-50HZ     ::= 0x05
  /** Output data rate of 25 Hz. */
  static ODR-25HZ     ::= 0x06
  /** Output data rate of 12.5 Hz. */
  static ODR-12-5HZ   ::= 0x07
  /** Output data rate of 6.25 Hz. */
  static ODR-6-25HZ   ::= 0x08
  /** Output data rate of 3.125 Hz. */
  static ODR-3-125HZ  ::= 0x09
  /** Output data rate of 1.5625 Hz. */
  static ODR-1-5625HZ ::= 0x0a

  /** No averaging. One sample per measurement. */
  static AVG-NONE      ::= 0x0
  /** Averages 2 samples per measurement. */
  static AVG-2-SAMPLES ::= 0x1
  /** Averages 4 samples per measurement. */
  static AVG-4-SAMPLES ::= 0x2
  /** Averages 8 samples per measurement. Lowest noise. */
  static AVG-8-SAMPLES ::= 0x3

  /** Power mode: no measurements, lowest power. */
  static MODE-SUSPEND     ::= 0x00
  /** Power mode: continuous measurements at the configured rate. */
  static MODE-NORMAL      ::= 0x01
  /** Power mode: one measurement per request, then back to suspend. */
  static MODE-FORCED      ::= 0x03
  /** Power mode: like $MODE-FORCED, but faster. Only for rates of 25 Hz and up. */
  static MODE-FORCED-FAST ::= 0x04

  /** Length of one sensortime tick, in microseconds. */
  static SENSORTIME-TICK-US ::= 39.0625

  static REG-CHIP-ID_           ::= 0x00
  static REG-ERR-REG_           ::= 0x02
  static REG-PAD-CTRL_          ::= 0x03
  static REG-PMU-CMD-AGGR-SET_  ::= 0x04
  static REG-PMU-CMD-AXIS-SET_  ::= 0x05
  static REG-PMU-CMD_           ::= 0x06
  static REG-PMU-CMD-STATUS-0_  ::= 0x07
  static REG-PMU-CMD-STATUS-1_  ::= 0x08
  static REG-I3C-ERR_           ::= 0x09
  static REG-I2C-WDT-SET_       ::= 0x0A
  static REG-INT-CTRL_          ::= 0x2E
  static REG-INT-CTRL-IBI_      ::= 0x2F
  static REG-INT-STATUS_        ::= 0x30
  static REG-MAG-X-XLSB_        ::= 0x31
  static REG-MAG-X-LSB_         ::= 0x32
  static REG-MAG-X-MSB_         ::= 0x33
  static REG-MAG-Y-XLSB_        ::= 0x34
  static REG-MAG-Y-LSB_         ::= 0x35
  static REG-MAG-Y-MSB_         ::= 0x36
  static REG-MAG-Z-XLSB_        ::= 0x37
  static REG-MAG-Z-LSB_         ::= 0x38
  static REG-MAG-Z-MSB_         ::= 0x39
  static REG-TEMP-XLSB_         ::= 0x3A
  static REG-TEMP-LSB_          ::= 0x3B
  static REG-TEMP-MSB_          ::= 0x3C
  static REG-SENSORTIME-XLSB_   ::= 0x3D
  static REG-SENSORTIME-LSB_    ::= 0x3E
  static REG-SENSORTIME-MSB_    ::= 0x3F
  static REG-OTP-CMD_           ::= 0x50
  static REG-OTP-DATA-MSB_      ::= 0x52
  static REG-OTP-DATA-LSB_      ::= 0x53
  static REG-OTP-STATUS_        ::= 0x55
  static REG-TMR-SELFTEST-USER_ ::= 0x60
  static REG-CTRL-USER_         ::= 0x61
  static REG-CMD_               ::= 0x7E

  // Masks: $REG-CHIP-ID_
  static CHIP-ID-FIXED-MASK_ ::= 0b11110000
  static CHIP-ID-OTP-MASK_   ::= 0b00001111

  // Masks: $REG-OTP-CMD_
  static OTP-CMD-CMD-MASK_  ::= 0b11100000
  static OTP-CMD-WORD-MASK_ ::= 0b00011111
  static OTP-CMD-DIR-READ_         ::= 0b001
  static OTP-CMD-DIR-PRGM-READ-1B_ ::= 0b010
  static OTP-CMD-DIR-PRGM_         ::= 0b011
  static OTP-CMD-PWR-OFF-OTP_      ::= 0b100  // Full width = 0x80.
  static OTP-CMD-EXT-READ_         ::= 0b101
  static OTP-CMD-EXT-PRGM_         ::= 0b111

  // Masks: $REG-ERR-REG_
  static ERR-REG-PMU-CMD-ERR_ ::= 0b00000001

  // Masks: $REG-INT-STATUS_
  static INT-STATUS-DRDY_ ::= 0b00000100

  // Masks: $REG-PMU-CMD-AGGR-SET_
  static PMU-CMD-AGGR-AVG-MASK_ ::= 0b00110000
  static PMU-CMD-AGGR-ODR-MASK_ ::= 0b00001111
  static AGGR-AVG-LOOKUP_ ::= {
    AVG-NONE: "None",
    AVG-2-SAMPLES: "2 Samples",
    AVG-4-SAMPLES: "4 Samples",
    AVG-8-SAMPLES: "8 Samples"}
  static AGGR-ODR-LOOKUP_ ::= {
    ODR-400HZ: "400Hz",
    ODR-200HZ: "200Hz",
    ODR-100HZ: "100Hz",
    ODR-50HZ: "50Hz",
    ODR-25HZ: "25Hz",
    ODR-12-5HZ: "12.5Hz",
    ODR-6-25HZ: "6.25Hz",
    ODR-3-125HZ: "3.125Hz",
    ODR-1-5625HZ: "1.5625Hz"}

  // Masks: $REG-PMU-CMD-AXIS-SET_
  static ENABLE-X_ ::= 0b00000001
  static ENABLE-Y_ ::= 0b00000010
  static ENABLE-Z_ ::= 0b00000100

  // Masks: $REG-PMU-CMD_
  static PMU-CMD-MODE-MASK_ ::= 0b00001111
  static PMU-CMD-MODE-SUSPEND_  ::= 0x00
  static PMU-CMD-MODE-NORMAL_   ::= 0x01
  static PMU-CMD-MODE-UPD-OAE_  ::= 0x02
  static PMU-CMD-MODE-FM_       ::= 0x03
  static PMU-CMD-MODE-FM-FAST_  ::= 0x04
  static PMU-CMD-MODE-FGR_      ::= 0x05
  static PMU-CMD-MODE-FGR-FAST_ ::= 0x06
  static PMU-CMD-MODE-BR_       ::= 0x07
  static PMU-CMD-MODE-BR-FAST_  ::= 0x08
  static PMU-CMD-MODE-LOOKUP_ ::= {
    PMU-CMD-MODE-SUSPEND_: "Suspend",
    PMU-CMD-MODE-NORMAL_: "Normal",
    PMU-CMD-MODE-UPD-OAE_: "Update OAE",
    PMU-CMD-MODE-FM_: "FM",
    PMU-CMD-MODE-FM-FAST_: "FM-Fast",
    PMU-CMD-MODE-FGR_: "FGR",
    PMU-CMD-MODE-FGR-FAST_: "FGR-Fast",
    PMU-CMD-MODE-BR_: "BR",
    PMU-CMD-MODE-BR-FAST_: "BR-Fast"}

  // Masks: $REG-PMU-CMD-STATUS-0_
  static CMD-STATUS-0-CMD-VALUE-MASK_ ::= 0b11100000  // Last command processed. Bosch reference code only.
  static CMD-STATUS-0-CMD-ILLEGAL_    ::= 0b00010000
  static CMD-STATUS-0-PWR-NORMAL_     ::= 0b00001000
  static CMD-STATUS-0-AVG-OVERWRITE_  ::= 0b00000100
  static CMD-STATUS-0-ODR-OVERWRITE_  ::= 0b00000010
  static CMD-STATUS-0-PMU-CMD-BUSY_   ::= 0b00000001

  // Masks: $REG-PMU-CMD-STATUS-1_
  static CMD-STATUS-1-AVG-EFFECTIVE_ ::= 0b00110000
  static CMD-STATUS-1-ODR-EFFECTIVE_ ::= 0b00001111

  // Masks: $REG-I3C-ERR_
  static I3C-ERR-ERR-3_ ::= 0b00001000  // S0/S1 Error.
  static I3C-ERR-ERR-0_ ::= 0b00000001  // SDR-Parity.

  // Masks: $REG-I2C-WDT-SET_: I2C watchdog configure registers
  static I2C-WDT-SEL_ ::= 0b00000010  // 0=short (1.28ms), 1=long (40.96ms)
  static I2C-WDT-EN_  ::= 0b00000001

  // Masks: $REG-INT-CTRL_
  static INT-CTRL-DATA-READY_    ::= 0b10000000  // 1=Data Ready on INT pin.
  static INT-CTRL-INT-OUTPUT-EN_ ::= 0b00001000  // Enable output on INT pin.
  static INT-CTRL-INT-OD_        ::= 0b00000100  // 0=Open drain, 1=Push pull.
  static INT-CTRL-INT-POL_       ::= 0b00000010  // 0=Act-low, 1=Act-high.
  static INT-CTRL-INT-MODE_      ::= 0b00000001  // 0=Pulsed, 1=Latched.

  // Masks: $REG-OTP-STATUS_
  static OTP-STATUS-ERR-MASK_       ::= 0b11100000
  static OTP-STATUS-CURR-PAGE-ADDR_ ::= 0b00011110
  static OTP-STATUS-CMD-DONE_       ::= 0b00000001
  static OTP-STATUS-ERR-NO-ERROR_   ::= 0b000
  static OTP-STATUS-ERR-BOOT-ERROR_ ::= 0b001
  static OTP-STATUS-ERR-PAGE-READ_  ::= 0b010
  static OTP-STATUS-ERR-PAGE-PRG_   ::= 0b011
  static OTP-STATUS-ERR-SIGN_       ::= 0b100
  static OTP-STATUS-ERR-INV-CMD_    ::= 0b101
  static OTP-STATUS-ERR-LOOKUP_ ::= {
    OTP-STATUS-ERR-NO-ERROR_: "No Error",
    OTP-STATUS-ERR-BOOT-ERROR_: "Boot Error",
    OTP-STATUS-ERR-PAGE-READ_: "Page Read Error",
    OTP-STATUS-ERR-PAGE-PRG_: "Page Program Error",
    OTP-STATUS-ERR-SIGN_: "Sign Error",
    OTP-STATUS-ERR-INV-CMD_: "Invalid Command Error"}

  // $write-register_ statics for bit width.  All 16 bit read/writes are LE.
  static WIDTH-8_ ::= 1
  static WIDTH-16_ ::= 2
  static DEFAULT-REGISTER-WIDTH_ ::= WIDTH-8_
  static DUMMY-BYTES_ ::= 2

  // Waits. The values come from the Bosch reference code (bmm350_defs.h).
  static WAIT-POR_               ::= Duration --ms=3
  static WAIT-SOFT-RESET_        ::= Duration --ms=24
  static WAIT-CHANGE-TO-SUSPEND_ ::= Duration --ms=6
  static WAIT-CHANGE-TO-NORMAL_  ::= Duration --ms=38
  static WAIT-ODR-UPDATE_        ::= Duration --ms=1
  static WAIT-BIT-RESET_         ::= Duration --ms=14
  static WAIT-FLUX-GUIDE-RESET_  ::= Duration --ms=18
  static DRDY-TIMEOUT_           ::= Duration --ms=1500
  static DRDY-POLL-MS_           ::= 1
  static BUSY-TIMEOUT_           ::= Duration --ms=500
  static OTP-TIMEOUT_            ::= Duration --ms=50

  // Wait from a forced mode request until data is ready, by averaging setting.
  static WAIT-FORCED-MS_      ::= [15, 17, 20, 28]
  static WAIT-FORCED-FAST-MS_ ::= [4, 5, 9, 16]

  static SIGNED-8-BIT_   ::= 8
  static SIGNED-12-BIT_  ::= 12
  static SIGNED-16-BIT_  ::= 16
  static SIGNED-21-BIT_  ::= 21
  static SIGNED-24-BIT_  ::= 24
  static SIGNED-DEFAULT_ ::= 0
  static SIGNED-MAP_     ::= {
    SIGNED-DEFAULT_: 0,
    SIGNED-8-BIT_:  "8 bit",
    SIGNED-12-BIT_: "12 bit",
    SIGNED-16-BIT_: "16 bit",
    SIGNED-21-BIT_: "21 bit",
    SIGNED-24-BIT_: "24 bit"}

  static OTP-DATA-LENGTH ::= 32
  static READ-BUFFER-LENGTH  ::= 127
  static MAG-TEMP-DATA-LEN ::= 12

  lsb-x_/float := 0.0070699787  // µT/LSB
  lsb-y_/float := 0.0070699787  // µT/LSB
  lsb-z_/float := 0.0071749641  // µT/LSB
  lsb-t_/float := 0.00098128185 // °C/LSB

  static SOFT-RESET-CMD_ ::= 0xB6

  // The OTP command byte that powers the OTP memory off (0x80).
  static OTP-POWER-OFF_ ::= OTP-CMD-PWR-OFF-OTP_ << 5

  dev_/serial.Device := ?
  reg_/serial.Registers := ?
  logger_/log.Logger := ?
  otp-data_/Bmm350Trim := ?
  axes_/int := ENABLE-X_ | ENABLE-Y_ | ENABLE-Z_

  /**
  Boots the chip and starts continuous measurement.

  Measurement runs in normal mode at the output data rate $odr, averaging
    $avg samples per measurement. Pass one of the ODR and AVG constants, such
    as $ODR-100HZ and $AVG-4-SAMPLES. The chip supports only some
    combinations, listed by $max-avg-for-odr. When $avg is higher than the rate
    allows, the driver lowers it to the highest value that works.

  Throws an exception if the chip does not answer with the BMM350 chip ID.
  */
  constructor device/serial.Device
      --odr/int=ODR-400HZ
      --avg/int=AVG-NONE
      --logger/log.Logger=log.default:
    dev_ = device
    reg_ = device.registers
    logger_ = logger.with-name "bmm350"
    otp-data_ = Bmm350Trim --logger=logger_

    // The boot sequence follows bmm350_init in the Bosch reference code.
    sleep WAIT-POR_
    reset_

    /*
    The chip ID is 0x33. The upper four bits are fixed and identify the
      BMM350. The lower four bits hold an OTP revision. The driver reads the OTP
      contents itself, so it works with any revision and checks only the upper
      four bits.
    */
    id := read-register_ REG-CHIP-ID_
    if (id >> 4) != 0b0011:
      throw "Unexpected chip id [$id]"
    logger_.info "BMM350 chip id" --tags={
      "id":"0x$(%02x id)",
      "fixed": bits-grouped_ (id >> 4) --min-display-bits=4,
      "otp": bits-grouped_ (id & 0b1111) --min-display-bits=4}

    /*
    After a boot the chip serves its trim values from OTP memory. The driver
      copies all 32 words into a $Bmm350Trim object, then powers the OTP off by
      writing 0x80 to $REG-OTP-CMD_. The OTP stays off until the next reset.
    */
    get-otp-data_
    write-register_ REG-OTP-CMD_ OTP-POWER-OFF_

    // The datasheet says the chip is ready to measure after this step.
    magnetic-reset

    configure-mag --avg=avg --odr=odr
    set-data-ready-check true
    set-mode MODE-NORMAL

  /**
  Reads the upper four bits of the chip ID.

  Returns 0b0011 (3) for a BMM350.
  */
  get-chip-id -> int:
    return read-register_ REG-CHIP-ID_ --mask=CHIP-ID-FIXED-MASK_

  /**
  Resets the chip and leaves it in suspend mode.

  All settings return to their power-on values, so call $configure-mag,
    $set-data-ready-check and $set-mode again to resume measuring. The trim
    values read at construction stay valid.
  */
  soft-reset -> none:
    reset_
    write-register_ REG-OTP-CMD_ OTP-POWER-OFF_
    magnetic-reset
    axes_ = ENABLE-X_ | ENABLE-Y_ | ENABLE-Z_

  // Sends the reset command and checks that the chip answers with its ID.
  reset_ -> none:
    write-register_ REG-CMD_ SOFT-RESET-CMD_
    sleep WAIT-SOFT-RESET_
    id := read-register_ REG-CHIP-ID_ --mask=CHIP-ID-FIXED-MASK_
    if id != 0b0011:
      throw "Unexpected chip id [$id]"

  /**
  Runs a magnetic reset: a bit reset followed by a flux guide reset.

  The chip does this on its own after every boot, and on every sample in
    normal and forced mode. Run it by hand after the sensor has sat in suspend
    mode through a very strong field, or before leaving a suspend of more than
    about two seconds (datasheet section 5.1.5). It takes roughly 35 ms.

  A chip in normal mode is suspended first and put back in normal mode
    afterwards.
  */
  magnetic-reset -> none:
    was-normal := is-pmu-pwr-normal_
    if was-normal: set-mode MODE-SUSPEND
    send-pmu-command_ PMU-CMD-MODE-BR_ WAIT-BIT-RESET_
    send-pmu-command_ PMU-CMD-MODE-FGR_ WAIT-FLUX-GUIDE-RESET_
    if was-normal: set-mode MODE-NORMAL

  /**
  Enables or disables the measurement of individual axes.

  Disabled axes read as 0.0 from $read-mag and $read-mag-compensated.

  The chip only accepts this change in suspend mode (datasheet section 5.7).
    A chip in normal mode is suspended first and put back afterwards.

  It is an error to disable all three axes.
  */
  enable-axis --x/bool=true --y/bool=true --z/bool=true -> none:
    raw := 0
    if x: raw = raw | ENABLE-X_
    if y: raw = raw | ENABLE-Y_
    if z: raw = raw | ENABLE-Z_
    if raw == 0: throw "At least one axis must stay enabled"
    was-normal := is-pmu-pwr-normal_
    if was-normal: set-mode MODE-SUSPEND
    write-register_ REG-PMU-CMD-AXIS-SET_ raw
    axes_ = raw
    if was-normal: set-mode MODE-NORMAL

  // Writes a PMU command, waits, and checks that the chip took it.
  send-pmu-command_ command/int wait/Duration -> none:
    write-register_ REG-PMU-CMD_ command
    sleep wait
    wait-pmu-idle_
    status := read-register_ REG-PMU-CMD-STATUS-0_
    done := (status & CMD-STATUS-0-CMD-VALUE-MASK_) >> CMD-STATUS-0-CMD-VALUE-MASK_.count-trailing-zeros
    if done != (command & 0b111):
      throw "BMM350 did not accept power command $command (status 0x$(%02x status))"

  // Waits until the chip has finished processing its last PMU command.
  wait-pmu-idle_ -> none:
    deadline := Time.monotonic-us + BUSY-TIMEOUT_.in-us
    while is-pmu-cmd-busy_:
      if Time.monotonic-us > deadline: throw "BMM350 busy timeout"
      sleep --ms=1

  /**
  Interprets the lowest $bits bits of $value as a signed number.
  */
  sign-extend_ value/int bits/int -> int:
    assert: SIGNED-MAP_.contains bits
    sign := 1 << (bits - 1)
    mask := (1 << bits) - 1
    v := value & mask
    return (v & sign) != 0 ? (v - (1 << bits)) : v

  /**
  Reads one measurement as raw bytes.

  The 15 bytes hold X, Y and Z (3 bytes each), temperature (3 bytes) and
    sensortime (3 bytes), in that order. They come from a single burst read, so
    they all belong to the same measurement (datasheet section 5.2). Pass them
    to $read-mag, $read-mag-compensated, $read-temp or $read-sensortime to
    convert them.

  With data ready reporting on, which is the default (see
    $set-data-ready-check), this waits for the next measurement first. It
    throws if none arrives within 1.5 seconds, which is what happens in suspend
    mode. With data ready reporting off, it returns whatever the data registers
    hold at that moment.
  */
  read-raw-data -> ByteArray:
    if is-data-ready-check-set: wait-data-ready_
    total-bytes := 15 + DUMMY-BYTES_
    return (reg_.read-bytes REG-MAG-X-XLSB_ total-bytes)[DUMMY-BYTES_..]

  /**
  Whether a new measurement is ready.

  The flag clears when read. It only works while data ready reporting is on, see
    $set-data-ready-check.
  */
  is-data-ready -> bool:
    return (read-register_ REG-INT-STATUS_ --mask=INT-STATUS-DRDY_) == 1

  /**
  Turns the data ready flag on or off.

  With the flag on, $is-data-ready reports new measurements and
    $read-raw-data waits for them. To also see the flag on the INT pin, turn
    the pin on with $set-int-pin.
  */
  set-data-ready-check enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-DATA-READY_

  /**
  Whether the data ready flag is turned on.
  */
  is-data-ready-check-set -> bool:
    return (read-register_ REG-INT-CTRL_ --mask=INT-CTRL-DATA-READY_) == 1

  /**
  Turns the INT pin output on or off.
  */
  set-int-pin enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-OUTPUT-EN_

  /**
  Whether the INT pin output is on.
  */
  is-int-pin-set -> bool:
    return (read-register_ REG-INT-CTRL_ --mask=INT-CTRL-INT-OUTPUT-EN_) == 1

  /**
  Sets the INT pin to push-pull when $push-pull is true, or open-drain when it
    is false.
  */
  set-int-pin-push-pull push-pull/bool -> none:
    value := 0
    if push-pull: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-OD_

  /**
  Sets the INT pin to active-high when $high is true, or active-low when it is
    false.
  */
  set-int-pin-polarity high/bool -> none:
    value := 0
    if high: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-POL_

  /**
  Sets the INT pin to stay asserted until the status is read when $latched is
    true, or to send a short pulse when it is false (the default).
  */
  set-int-latched-mode latched/bool -> none:
    value := 0
    if latched: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-MODE_

  /**
  Reads the magnetic field in µT, without temperature compensation.

  Use $read-mag-compensated for readings that are meant to be accurate. This
    version is cheaper and is useful for comparing against the compensated
    values. Disabled axes (see $enable-axis) read as 0.0.
  */
  read-mag bytes/ByteArray=read-raw-data -> Point3f:
    counts := mag-uncompensated_ bytes
    return Point3f counts[0] counts[1] counts[2]

  /**
  Reads the magnetic field in µT, corrected for the sensor's offset, gain,
    temperature drift and cross-axis effects.

  The corrections use the trim values stored in the chip's OTP memory. The
    math follows the Bosch reference code. Disabled axes (see $enable-axis)
    read as 0.0.
  */
  read-mag-compensated bytes/ByteArray=read-raw-data -> Point3f:
    counts := mag-uncompensated_ bytes
    t := temperature-uncompensated_ bytes
    out := otp-data_.apply-compensation counts[0] counts[1] counts[2] t
    x := (axes_ & ENABLE-X_) != 0 ? out[0] : 0.0
    y := (axes_ & ENABLE-Y_) != 0 ? out[1] : 0.0
    z := (axes_ & ENABLE-Z_) != 0 ? out[2] : 0.0
    return Point3f x y z

  // The three axes in µT, before compensation. Disabled axes count as zero.
  mag-uncompensated_ bytes/ByteArray -> List:
    x24 := bytes[0] + (bytes[1] << 8) + (bytes[2] << 16)
    y24 := bytes[3] + (bytes[4] << 8) + (bytes[5] << 16)
    z24 := bytes[6] + (bytes[7] << 8) + (bytes[8] << 16)

    // The registers hold 21-bit signed values (datasheet section 4).
    x := (axes_ & ENABLE-X_) != 0 ? (sign-extend_ x24 SIGNED-21-BIT_) * lsb-x_ : 0.0
    y := (axes_ & ENABLE-Y_) != 0 ? (sign-extend_ y24 SIGNED-21-BIT_) * lsb-y_ : 0.0
    z := (axes_ & ENABLE-Z_) != 0 ? (sign-extend_ z24 SIGNED-21-BIT_) * lsb-z_ : 0.0
    return [x, y, z]

  /**
  Reads the sensortime of a measurement, in ticks.

  Each tick is $SENSORTIME-TICK-US microseconds. The value comes from a 24-bit
    unsigned counter that wraps around, and it only advances while the chip is
    in normal mode. To compare two readings across a wrap-around, use
    $sensortime-elapsed.
  */
  read-sensortime bytes/ByteArray=read-raw-data -> int:
    return bytes[12] + (bytes[13] << 8) + (bytes[14] << 16)

  /**
  Returns the ticks between two sensortime readings, $earlier and $later.

  Handles the counter wrapping around between the two.
  */
  static sensortime-elapsed earlier/int later/int -> int:
    return (later - earlier) & 0xFFFFFF

  /**
  Converts a number of sensortime $ticks to microseconds.
  */
  static sensortime-to-us ticks/int -> float:
    return ticks * SENSORTIME-TICK-US

  /**
  Reads the die temperature in °C.

  Applies the temperature trim stored in OTP, as the Bosch reference code does.
  */
  read-temp bytes/ByteArray=read-raw-data -> float:
    return otp-data_.compensate-temperature (temperature-uncompensated_ bytes)

  // The temperature in °C before the OTP trim, with the fixed correction from
  // the Bosch reference code applied.
  temperature-uncompensated_ bytes/ByteArray -> float:
    t24 := bytes[9] + (bytes[10] << 8) + (bytes[11] << 16)
    temp := (sign-extend_ t24 SIGNED-24-BIT_) * lsb-t_
    if temp > 0.0:
      temp = temp - 25.49
    else if temp < 0.0:
      temp = temp + 25.49
    return temp

  // Reads all 32 OTP words into the trim object.
  get-otp-data_ -> none:
    otp-cmd := OTP-CMD-DIR-READ_ << OTP-CMD-CMD-MASK_.count-trailing-zeros
    OTP-DATA-LENGTH.repeat: | word-index |
      write-register_ REG-OTP-CMD_ (otp-cmd | (word-index & OTP-CMD-WORD-MASK_))
      wait-otp-done_
      otp-lsb := read-register_ REG-OTP-DATA-LSB_
      otp-msb := read-register_ REG-OTP-DATA-MSB_
      otp-data_[word-index] = #[otp-lsb, otp-msb]

  // Waits for the chip to finish an OTP command, and throws on an OTP error.
  wait-otp-done_ -> none:
    deadline := Time.monotonic-us + OTP-TIMEOUT_.in-us
    while true:
      raw := read-register_ REG-OTP-STATUS_
      error := (raw & OTP-STATUS-ERR-MASK_) >> OTP-STATUS-ERR-MASK_.count-trailing-zeros
      if error != OTP-STATUS-ERR-NO-ERROR_:
        text := OTP-STATUS-ERR-LOOKUP_.get error --if-absent=: "Unknown error"
        throw "BMM350 OTP error: $text"
      if (raw & OTP-STATUS-CMD-DONE_) != 0: return
      if Time.monotonic-us > deadline: throw "BMM350 OTP read timeout"
      sleep --ms=1

  is-pmu-cmd-illegal_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-CMD-ILLEGAL_) == 1

  is-pmu-pwr-normal_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-PWR-NORMAL_) == 1

  is-pmu-avg-overwrite_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-AVG-OVERWRITE_) == 1

  is-pmu-odr-overwrite_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-ODR-OVERWRITE_) == 1

  is-pmu-cmd-busy_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-PMU-CMD-BUSY_) == 1

  /**
  Whether the chip has flagged a power command error.

  The flag clears when read.
  */
  is-pmu-error -> bool:
    return (read-register_ REG-ERR-REG_ --mask=ERR-REG-PMU-CMD-ERR_) == 1

  /**
  Sets the power mode to $MODE-NORMAL, $MODE-SUSPEND, $MODE-FORCED or
    $MODE-FORCED-FAST.

  A chip in normal mode only accepts a switch to suspend (datasheet section
    5.1.3), so this suspends it first when needed. The call returns once the
    mode change is done: about 38 ms for normal mode, and between 4 and 28 ms
    for a forced measurement, depending on averaging.

  In forced mode the chip takes one measurement and returns to suspend.
    Read the result with $read-raw-data afterwards.
  */
  set-mode mode/int=MODE-NORMAL -> none:
    if mode != MODE-SUSPEND and mode != MODE-NORMAL and mode != MODE-FORCED and mode != MODE-FORCED-FAST:
      throw "Unsupported power mode $mode"

    if is-pmu-pwr-normal_:
      write-register_ REG-PMU-CMD_ PMU-CMD-MODE-SUSPEND_
      sleep WAIT-CHANGE-TO-SUSPEND_
      wait-pmu-idle_
      // Reading the status clears a data ready flag left over from normal mode.
      // Without this, the first forced measurement would look ready too early.
      read-register_ REG-INT-STATUS_
    if mode == MODE-SUSPEND:
      logger_.debug "mode configured" --tags={"mode":PMU-CMD-MODE-LOOKUP_[mode]}
      return

    write-register_ REG-PMU-CMD_ mode
    if mode == MODE-NORMAL:
      sleep WAIT-CHANGE-TO-NORMAL_
    else:
      avg := read-register_ REG-PMU-CMD-AGGR-SET_ --mask=PMU-CMD-AGGR-AVG-MASK_
      sleep --ms=(mode == MODE-FORCED ? WAIT-FORCED-MS_[avg] : WAIT-FORCED-FAST-MS_[avg])
    wait-pmu-idle_
    logger_.debug "mode configured" --tags={"mode":PMU-CMD-MODE-LOOKUP_[mode]}

  /**
  Reads the output data rate the chip is using right now.

  Returns one of the ODR constants, such as $ODR-100HZ.
  */
  get-odr -> int:
    odr := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-ODR-EFFECTIVE_
    text := AGGR-ODR-LOOKUP_.get odr --if-absent=: "unknown"
    logger_.debug "effective odr" --tags={"odr":"$(%02x odr)","text":text}
    return odr

  /**
  Reads the averaging the chip is using right now.

  Returns one of the AVG constants, such as $AVG-4-SAMPLES.
  */
  get-avg -> int:
    avg := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-AVG-EFFECTIVE_
    text := AGGR-AVG-LOOKUP_.get avg --if-absent=: "unknown"
    logger_.debug "effective avg" --tags={"avg":"$(%02x avg)","text":text}
    return avg

  /**
  Returns the highest averaging the chip allows at the output data rate $odr.

  The chip allows no averaging at 400 Hz, up to $AVG-2-SAMPLES at 200 Hz, up to
    $AVG-4-SAMPLES at 100 Hz, and any averaging at 50 Hz or slower (datasheet
    Table 5).
  */
  static max-avg-for-odr odr/int -> int:
    if odr <= ODR-400HZ: return AVG-NONE
    if odr == ODR-200HZ: return AVG-2-SAMPLES
    if odr == ODR-100HZ: return AVG-4-SAMPLES
    return AVG-8-SAMPLES

  /**
  Returns the rate in Hz for one of the ODR constants.
  */
  static odr-to-hz odr/int -> float:
    assert: ODR-400HZ <= odr <= ODR-1-5625HZ
    return 1600.0 / (1 << odr)

  /**
  Sets the output data rate to $odr.

  Keeps the current averaging when the new rate allows it, and lowers it
    otherwise.
  */
  set-odr odr/int=ODR-50HZ -> none:
    avg := read-register_ REG-PMU-CMD-AGGR-SET_ --mask=PMU-CMD-AGGR-AVG-MASK_
    configure-mag --avg=avg --odr=odr

  /**
  Sets the averaging to $avg.

  Keeps the current output data rate. When the rate does not allow $avg, the
    averaging is lowered to the highest value it does allow (see
    $max-avg-for-odr).
  */
  set-avg avg/int=AVG-NONE -> none:
    odr := read-register_ REG-PMU-CMD-AGGR-SET_ --mask=PMU-CMD-AGGR-ODR-MASK_
    configure-mag --avg=avg --odr=odr

  /**
  Sets the output data rate and the averaging together.

  Averaging higher than $odr allows is lowered to the highest value that works,
    as the datasheet describes (section 5.4). Read $get-avg afterwards to see
    what the chip uses. Setting both in one call means the chip sees a single
    update.
  */
  configure-mag --avg/int=AVG-NONE --odr/int=ODR-50HZ -> none:
    if not AGGR-ODR-LOOKUP_.contains odr: throw "Invalid output data rate $odr"
    if not AGGR-AVG-LOOKUP_.contains avg: throw "Invalid averaging $avg"
    allowed := max-avg-for-odr odr
    if avg > allowed:
      logger_.debug "averaging lowered to fit the output data rate" --tags={
        "odr": AGGR-ODR-LOOKUP_[odr],
        "requested": AGGR-AVG-LOOKUP_[avg],
        "used": AGGR-AVG-LOOKUP_[allowed]}
      avg = allowed
    write-register_ REG-PMU-CMD-AGGR-SET_ ((avg << PMU-CMD-AGGR-AVG-MASK_.count-trailing-zeros) | odr)
    update-aggr-values_

  /*
  Applies new ODR and averaging settings with the update command.

  The datasheet says that after an update, the next update must wait for a data
    ready that comes after the busy flag clears: "looking only at the busy flag
    is not correct because the busy flag can be deasserted too early" (PMU_CMD
    0x02, register 8.6).
  */
  update-aggr-values_ -> none:
    write-register_ REG-PMU-CMD_ PMU-CMD-MODE-UPD-OAE_
    sleep WAIT-ODR-UPDATE_
    wait-pmu-idle_
    if not is-pmu-pwr-normal_: return
    if is-data-ready-check-set:
      wait-data-ready_
    else:
      odr := read-register_ REG-PMU-CMD-AGGR-SET_ --mask=PMU-CMD-AGGR-ODR-MASK_
      sleep --ms=((2000.0 / (odr-to-hz odr)).to-int + 1)

  // Waits for the next data ready flag. Throws after the data ready timeout.
  wait-data-ready_ -> none:
    deadline := Time.monotonic-us + DRDY-TIMEOUT_.in-us
    while not is-data-ready:
      if Time.monotonic-us > deadline:
        throw "BMM350 data ready timeout (is it in suspend mode?)"
      sleep --ms=DRDY-POLL-MS_

  /*
  Reads and optionally masks and parses register data (little-endian).

  The chip sends two dummy bytes (DUMMY-BYTES_) ahead of the data on every read,
    so the read asks for two extra bytes and drops them. Writes do not have
    dummy bytes.
  */
  read-register_ -> int
      register/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false:
    assert: (width == WIDTH-8_) or (width == WIDTH-16_)
    total-bytes := width + DUMMY-BYTES_

    if not mask: mask = (width == WIDTH-8_) ? 0xFF : 0xFFFF
    if not offset: offset = mask.count-trailing-zeros

    if width == WIDTH-8_: assert: (mask & ~0xFF) == 0
    else: assert: (mask & ~0xFFFF) == 0
    assert: mask != 0

    full-width := (offset == 0) and ((width == WIDTH-8_ and mask == 0xFF) or (width == WIDTH-16_ and mask == 0xFFFF))
    if signed and not full-width:
      throw "masked signed read not supported (need sign-extension by field width)"

    register-bytes := reg_.read-bytes register total-bytes
    register-value/int := ?
    if width == WIDTH-8_:
      register-value = signed ? io.LITTLE-ENDIAN.int8 register-bytes DUMMY-BYTES_ : register-bytes[DUMMY-BYTES_]
    else:
      register-value = signed ? io.LITTLE-ENDIAN.int16 register-bytes DUMMY-BYTES_ : io.LITTLE-ENDIAN.uint16 register-bytes DUMMY-BYTES_

    if full-width:
      return register-value

    return (register-value & mask) >> offset

  /*
  Writes register data, either as a full register write or as a masked
    read-modify-write (little-endian).
  */
  write-register_ -> none
      register/int
      value/int
      --mask/int?=null
      --offset/int?=null
      --width/int=DEFAULT-REGISTER-WIDTH_
      --signed/bool=false:
    assert: (width == WIDTH-8_) or (width == WIDTH-16_)

    if not mask: mask = (width == WIDTH-8_) ? 0xFF : 0xFFFF
    if not offset: offset = mask.count-trailing-zeros

    // Check that the mask fits the register width.
    if width == WIDTH-8_: assert: (mask & ~0xFF) == 0
    else: assert: (mask & ~0xFFFF) == 0

    // Work out whether this is a full-width write.
    full-width := (offset == 0) and ((width == WIDTH-8_ and mask == 0xFF) or (width == WIDTH-16_ and mask == 0xFFFF))

    // For now don't accept negative numbers as masked writes.
    if signed and not full-width:
      throw "masked signed write not supported (encode to field bits first)"

    // The field must fit within the register width.
    field-mask/int := mask >> offset
    assert: field-mask != 0

    // Check that the value fits the field.
    if not signed:
      assert: value >= 0 and value <= field-mask
    else:
      if width == WIDTH-8_: assert: -128 <= value and value <= 127
      else: assert: -32768 <= value and value <= 32767

    // Full-width writes go straight to the register.
    if full-width:
      if width == WIDTH-8_:
        signed ? reg_.write-i8 register value : reg_.write-u8 register value
      else:
        signed ? reg_.write-i16-le register value : reg_.write-u16-le register value
      return

    // The read half of a masked write must skip the dummy bytes too.
    old-value/int := read-register_ register --width=width
    reg-mask/int := (width == WIDTH-8_) ? 0xFF : 0xFFFF
    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset) & reg-mask

    // Write the modified value.
    if width == WIDTH-8_:
      reg_.write-u8 register new-value
    else:
      reg_.write-u16-le register new-value

  /**
  Provides strings to display bitmasks nicely when testing.
  */
  bits-grouped_ x/int
      --min-display-bits/int=0
      --group-size/int=4
      --sep/string="."
      -> string:

    assert: x >= 0
    assert: group-size > 0

    // Raw binary.
    bin := "$(%b x)"

    // Choose the width: at least min-display-bits, rounded up to a full group.
    groups := 0
    leftover := 0
    width := bin.size
    if min-display-bits > width:
      width = min-display-bits
    if group-size > width:
      width = group-size
    leftover = width % group-size
    if leftover > 0:
      width = width + (group-size - leftover)

    // Left-pad to the chosen width.
    bin = bin.pad --left width '0'

    // Group from left to right.
    out := ""
    i := 0
    while i < bin.size:
      if i > 0: out = "$(out)$(sep)"
      j := i + group-size
      if j > bin.size: j = bin.size
      out = "$(out)$(bin[i..j])"
      i = j

    return out

/**
The factory trim values of a BMM350, and the math that applies them.

The chip keeps 32 16-bit words of trim data in OTP memory. The getters decode
  those words into the offsets, sensitivities, temperature coefficients and
  cross-axis factors that the compensation needs. The decoding and the math
  follow the Bosch reference code (update_mag_off_sens and
  bmm350_get_compensated_mag_xyz_temp_data).

The $Bmm350 class fills one of these from the chip at construction. Tests can
  fill one by hand with the `[]=` operator.
*/
class Bmm350Trim:
  payload_/List := ?
  ready_/bool := false
  logger_/log.Logger := ?

  constructor --logger/log.Logger=log.default: // Empty
    payload_ = List Bmm350.OTP-DATA-LENGTH
    logger_ = logger.with-name "compensator"

  constructor.from-list_ data/List --logger/log.Logger=log.default:
    assert: data.size == Bmm350.OTP-DATA-LENGTH
    payload_ = data
    logger_ = logger.with-name "compensator"

  operator [] n/int -> int:
    assert: 0 <= n <= (Bmm350.OTP-DATA-LENGTH - 1)
    return payload_[n]

  operator []= n/int input/any -> none:
    assert: 0 <= n <= (Bmm350.OTP-DATA-LENGTH - 1)
    if input is int:
      payload_[n] = input
    else if input is ByteArray:
      assert: input.size == 2
      payload_[n] = (input[1] << 8) | input[0]
    else:
      throw "unhandled input type"

  sign8_ v/int -> int:
    v = v & 0xFF
    return (v & 0x80) != 0 ? (v - 0x100) : v

  sign12_ v/int -> int:
    v = v & 0x0FFF
    return (v & 0x0800) != 0 ? (v - 0x1000) : v

  sign16_ v/int -> int:
    v = v & 0xFFFF
    return (v & 0x8000) != 0 ? (v - 0x10000) : v

  // Temperature offset/sensitivity, one word in two parts.
  offset-t -> float: return (sign8_ (payload_[0x0D] & 0x00FF)) / 5.0
  sens-t -> float: return (sign8_ ((payload_[0x0d] >> 8) & 0x00FF)) / 512.0

  // Offsets packed across words, signed 12-bit.
  offset-x -> int: return sign12_ (payload_[0x0e] & 0x0FFF)
  offset-y -> int: return sign12_ (((payload_[0x0e] & 0xF000) >> 4) + (payload_[0x0f] & 0x00FF))
  offset-z -> int: return sign12_ ((payload_[0x0f] & 0x0F00) + (payload_[0x10] & 0x00FF))

  // Sensitivities, signed 8-bit.
  sens-x -> float: return (sign8_ ((payload_[0x10] >> 8) & 0x00FF)) / 256.0
  sens-y -> float: return (sign8_ (payload_[0x11] & 0x00FF)) / 256.0 + 0.01
  sens-z -> float: return (sign8_ ((payload_[0x11] >> 8) & 0x00FF)) / 256.0

  // Temp coefficients TCO/TCS.
  temp-co-x -> float: return (sign8_ (payload_[0x12] & 0x00FF)) / 32.0
  temp-cs-x -> float: return (sign8_ ((payload_[0x12] >> 8) & 0x00FF)) / 16384.0

  temp-co-y -> float: return (sign8_ (payload_[0x13] & 0x00FF)) / 32.0
  temp-cs-y -> float: return (sign8_ ((payload_[0x13] >> 8) & 0x00FF)) / 16384.0

  temp-co-z -> float: return (sign8_ (payload_[0x14] & 0x00FF)) / 32.0
  temp-cs-z -> float: return (sign8_ ((payload_[0x14] >> 8) & 0x00FF)) / 16384.0 - 0.0001

  // Cross-axis.
  cross-x-y -> float: return (sign8_ (payload_[0x15] & 0x00FF)) / 800.0
  cross-y-x -> float: return (sign8_ ((payload_[0x15] >> 8) & 0x00FF)) / 800.0
  cross-z-x -> float: return (sign8_ (payload_[0x16] & 0x00FF)) / 800.0
  cross-z-y -> float: return (sign8_ ((payload_[0x16] >> 8) & 0x00FF)) / 800.0

  // Reference temperature.
  temp-reference -> float: return (sign16_ (payload_[0x18])) / 512.0 + 23.0

  /**
  Applies the temperature trim to a temperature $t in °C.
  */
  compensate-temperature t/float -> float:
    return (1.0 + sens-t) * t + offset-t

  /**
  Variant of $(apply-compensation x y z t) that takes the field as a point.
  */
  apply-compensation point/Point3f t/float -> List:
    return apply-compensation point.x point.y point.z t

  /**
  Applies the trim to a field reading and a temperature.

  Takes the uncompensated field $x, $y and $z in µT and the temperature $t in
    °C. Returns a list with the compensated X, Y and Z in µT, followed by the
    compensated temperature in °C.
  */
  apply-compensation x/float y/float z/float t/float -> List:
    // Temp compensation.
    t = compensate-temperature t
    dt := t - temp-reference

    // Axis compensation.
    x = x * (1.0 + sens-x) + offset-x + temp-co-x * dt
    x = x / (1.0 + temp-cs-x * dt)

    y = y * (1.0 + sens-y) + offset-y + temp-co-y * dt
    y = y / (1.0 + temp-cs-y * dt)

    z = z * (1.0 + sens-z) + offset-z + temp-co-z * dt
    z = z / (1.0 + temp-cs-z * dt)

    // Cross-axis compensation.
    den := 1.0 - cross-y-x * cross-x-y
    if den == 0: den = 1e-9

    x2 := (x - cross-x-y * y) / den
    y2 := (y - cross-y-x * x) / den

    z2 := z + (x * (cross-y-x * cross-z-y - cross-z-x) -
        y * (cross-z-y - cross-x-y * cross-z-x)) / den

    return [x2, y2, z2, t]

  /** The 32 words as hex, for logging. */
  stringify -> string:
    out-list := List Bmm350.OTP-DATA-LENGTH
    payload_.size.repeat: | index |
      out-list[index] = "0x$(%04x payload_[index])"
    return "[$(out-list.join ", ")]"

  /** Logs every decoded trim value at debug level. */
  debug-log -> none:
    logger_.debug "data" --tags={
      "off-t": offset-t,
      "sens-t": sens-t,
      "t0": temp-reference,
      "off-x": offset-x,
      "off-y": offset-y,
      "off-z": offset-z,
      "sens-x": sens-x,
      "sens-y": sens-y,
      "sens-z": sens-z,
      "tco-x": temp-co-x,
      "tco-y": temp-co-y,
      "tco-z": temp-co-z,
      "tcs-x": temp-cs-x,
      "tcs-y": temp-cs-y,
      "tcs-z": temp-cs-z,
      "c-xy": cross-x-y,
      "c-yx": cross-y-x,
      "c-zx": cross-z-x,
      "c-zy": cross-z-y}
