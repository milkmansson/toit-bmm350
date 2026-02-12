// Copyright (C) 2026 Toit Contributors
// Use of this source code is governed by an MIT-style license that can be
// found in the package's LICENSE file.

import io
import serial
import log
import math show *

/**
Toit driver for Bosch Sensortec BST-BMM350 Magnetometer.

Supports I2C and I3C. Only I2C is currently tested/implemented, I3C has not
  been tested at all.
*/
/*
On I2C reads (and also I3C reads) the BMM350 returns two dummy bytes first,
  then the actual register data. Bosch explicitly states that for an n-byte
  read you must actually read n+2 bytes and discard the first 2.  This has
  been implemented in $read-register_ and $write-register_.
*/
class Bmm350:
  static I2C-ADDRESS      ::= 0x14       // Pin ADSEL Low.
  static I2C-ADDRESS-ALT  ::= 0x15   // Pin ADSEL high.

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
  static AGGR-AVG-NONE_      ::= 0x0
  static AGGR-AVG-2-SAMPLES_ ::= 0x1
  static AGGR-AVG-4-SAMPLES_ ::= 0x2
  static AGGR-AVG-8-SAMPLES_ ::= 0x3
  static AGGR-AVG-LOOKUP_ ::= {
    AGGR-AVG-NONE_: "None",
    AGGR-AVG-2-SAMPLES_: "2 Samples",
    AGGR-AVG-4-SAMPLES_: "4 Samples",
    AGGR-AVG-8-SAMPLES_: "8 Samples"}
  static AVG-CLAMP-ODR-LOOKUP_ ::= {
    AGGR-AVG-4-SAMPLES_: AGGR-ODR-100HZ_,
    AGGR-AVG-2-SAMPLES_: AGGR-ODR-200HZ_,
    AGGR-AVG-NONE_: AGGR-ODR-400HZ_}

  static AGGR-ODR-400HZ_    ::= 0x02
  static AGGR-ODR-200HZ_    ::= 0x03
  static AGGR-ODR-100HZ_    ::= 0x04
  static AGGR-ODR-50HZ_     ::= 0x05
  static AGGR-ODR-25HZ_     ::= 0x06
  static AGGR-ODR-12-5HZ_   ::= 0x07
  static AGGR-ODR-6-25HZ_   ::= 0x08
  static AGGR-ODR-3-125HZ_  ::= 0x09
  static AGGR-ODR-1-5625HZ_ ::= 0x0a
  static AGGR-ODR-LOOKUP_ ::= {
    AGGR-ODR-400HZ_: "400Hz",
    AGGR-ODR-200HZ_: "200Hz",
    AGGR-ODR-100HZ_: "100Hz",
    AGGR-ODR-50HZ_: "50Hz",
    AGGR-ODR-25HZ_: "25Hz",
    AGGR-ODR-12-5HZ_: "12.5Hz",
    AGGR-ODR-6-25HZ_: "6.25Hz",
    AGGR-ODR-3-125HZ_: "3.125Hz",
    AGGR-ODR-1-5625HZ_: "1.5625Hz"}
  static ODR-CLAMP-AVG-LOOKUP_ ::= {
    AGGR-ODR-100HZ_: AGGR-AVG-4-SAMPLES_,
    AGGR-ODR-200HZ_: AGGR-AVG-2-SAMPLES_,
    AGGR-ODR-400HZ_: AGGR-AVG-NONE_}

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
  static CMD-STATUS-0-CMD-ILLEGAL_    ::= 0b00010000
  static CMD-STATUS-0-PWR-NORMAL_     ::= 0b00001000
  static CMD-STATUS-0-AVG-OVWERWRITE_ ::= 0b00000100
  static CMD-STATUS-0-ODR-OVWERWRITE_ ::= 0b00000010
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

  // Sleeps:
  static WAIT-POR_               ::= Duration --ms=3
  static WAIT-SOFT-RESET_        ::= Duration --ms=24    // datasheet says 24?
  static WAIT-REQUEST-SUSPEND_   ::= Duration --ms=6
  static WAIT-CHANGE-TO-SUSPEND_ ::= Duration --ms=6
  static WAIT-CHANGE-TO-NORMAL_  ::= Duration --ms=38
  static WAIT-SUSPEND-TO-FORCED_ ::= Duration --ms=28  // 4..28 ms depending on config
  static WAIT-ODR-UPDATE_        ::= Duration --ms=1
  static WAIT-BR-FGR-RESET_      ::= Duration --ms=18  // 14-18 ms
  static DRDY-TIMEOUT_           ::= Duration --ms=1500

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

  dev_/serial.Device := ?
  reg_/serial.Registers := ?
  logger_/log.Logger := ?
  otp-data_ := ?

  constructor device/serial.Device --logger/log.Logger=log.default:
    dev_ = device
    reg_ = device.registers
    logger_ = logger.with-name "bmm350"
    otp-data_ = Bmm350Trim --logger=logger_

    // Bringup as ported from Bosch github (see README.md).
    // 1. Sleep after POR:
    sleep WAIT-POR_

    // 2. Soft Reset + delay:
    soft-reset

    // 4. Read Chip ID + check it:
    id := read-register_ REG-CHIP-ID_

    /*
    0x33 (00110011) = full width read for REG-CHIP-ID_, and is the Datasheet
      expected Chip ID number.
    'Fixed' portion of the mask is in the  higher 4 bits, eg, 0b0011.
    'NV' or changeable portion is the lower 4 bits - a version number of the OTP
      revision, some sort and may change based on otp content.  Since we read
      OTP content and use it in calculations in software, this driver would
      still be valid regardless of if the user programs it resulting in the
      lower 4 bits changing.  Therefore we only check the upper 4 bits for the
      device model.
    */
    if (id >> 4) != 0b0011:
      throw "Unexpected chip id [$id]"
    logger_.info "BMM350 chip id" --tags={
      "id":"0x$(%02x id)",
      "fixed": bits-grouped_ (id >> 4) --min-display-bits=4,
      "otp": bits-grouped_ (id & 0b1111) --min-display-bits=4}

    /*
    Datasheet: On boot, compensation coefficients are retrieved from One-Time
      -Programmable (OTP) memory.  The boot phase must be terminated by writing
      0x80 to $REG-OTP-CMD_.  From then on the OTP is inaccessible to the API
      unless a power reset or soft reset triggers another boot.  In this driver
      the data is obtained and put into a $Bmm350Trim object for continued use
      after the boot process has finished.
    */
    get-otp-data_
    write-register_ REG-OTP-CMD_ OTP-CMD-PWR-OFF-OTP_ --mask=OTP-CMD-CMD-MASK_

    // Configure Averaging and Output Data Rate (ODR)
    configure-mag --avg=AGGR-AVG-8-SAMPLES_ --odr=AGGR-ODR-400HZ_

    // Enable 'Data Ready'
    set-data-ready-check true

    // Kick off Normal mode:
    set-mode PMU-CMD-MODE-NORMAL_

  /**
  Reads Chip ID to determine what the chip is.
  */
  get-chip-id -> int:
    return read-register_ REG-CHIP-ID_ --mask=CHIP-ID-FIXED-MASK_

  /**
  Reset.
  */
  soft-reset -> none:
    write-register_ REG-CMD_ 0xb6
    sleep WAIT-SOFT-RESET_
    write-register_ REG-CMD_ 0x0
    sleep WAIT-SOFT-RESET_
    id := read-register_ REG-CHIP-ID_ --mask=CHIP-ID-FIXED-MASK_
    if id != 0b0011:
      throw "Unexpected chip id [$id]"

  /**
  Enable and disable specific axes.
  */
  enable-axis --x/bool=true --y/bool=true --z/bool=true -> none:
    raw := 0
    if x: raw = raw | ENABLE-X_
    if y: raw = raw | ENABLE-Y_
    if z: raw = raw | ENABLE-Z_
    write-register_ REG-PMU-CMD-AXIS-SET_ raw

  /**
  Extends the sign bit on bit 21 from the raw data to toit integer sign.
  */
  sign-extend_ value/int bits/int -> int:
    assert: SIGNED-MAP_.contains bits
    sign := 1 << (bits - 1)
    mask := (1 << bits) - 1
    v := value & mask
    return (v & sign) != 0 ? (v - (1 << bits)) : v

  /**
  Reads one instance of raw data.

  If read, can be manually supplied to the other read functions and converted,
    ensuring the reading of all data from that one measurement in time.
  */
  read-raw-data -> ByteArray:
    // Wait for ready, but with a timeout.
    exception := catch:
      with-timeout DRDY-TIMEOUT_:
          while not is-data-ready:
            sleep --ms=10

    if exception:
      logger_.error "read-raw-data timed out" --tags={"timeout-ms":DRDY-TIMEOUT_.in-ms}
      if not is-data-ready-check-set:
        throw "Data wasn't ready by timeout $DRDY-TIMEOUT_.in-ms ms, and set-data-ready-check not set."

    total-bytes := 15 + DUMMY-BYTES_
    read-bytes := (reg_.read-bytes REG-MAG-X-XLSB_ total-bytes)[DUMMY-BYTES_..]
    return read-bytes

  /**
  Whether new data ready.

  Clears when read.  Requires $set-data-ready-check to be true.
  */
  is-data-ready -> bool:
    return (read-register_ REG-INT-STATUS_) != 0

  /**
  Enables 'Mag Data Ready' interrupt, onto INT pin and $REG-INT-STATUS_ register.
  */
  set-data-ready-check enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-DATA-READY_

  /**
  Whether 'Mag Data Ready' flag is enabled.
  */
  is-data-ready-check-set -> bool:
    return (read-register_ REG-INT-CTRL_ --mask=INT-CTRL-DATA-READY_) == 1

  /**
  Enables Interrupt Pin.
  */
  set-int-pin enabled/bool -> none:
    value := 0
    if enabled: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-OUTPUT-EN_

  /**
  Whether Interrupt Output is enabled on it's pin.
  */
  is-int-pin-set -> bool:
    return (read-register_ REG-INT-CTRL_ --mask=INT-CTRL-INT-OUTPUT-EN_) == 1

  /**
  Sets Interrupt Pin to Open-Drain (false) or Push-Pull (true).
  */
  set-int-pin-push-pull push-pull/bool -> none:
    value := 0
    if push-pull: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-OD_

  /**
  Sets Interrupt Pin polarity.
  */
  set-int-pin-polarity high/bool -> none:
    value := 0
    if high: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-POL_

  /**
  Sets Interrupt latching (true) or pulsing (false, default).
  */
  set-int-latched-mode latched/bool -> none:
    value := 0
    if latched: value = 1
    write-register_ REG-INT-CTRL_ value --mask=INT-CTRL-INT-POL_

  /**
  Read Magnetometer.
  */
  read-mag bytes/ByteArray=read-raw-data -> Point3f:
    // x=3, y=3, z=3, t=3

    x24 := bytes[0] + (bytes[1] << 8) + (bytes[2] << 16)
    y24 := bytes[3] + (bytes[4] << 8) + (bytes[5] << 16)
    z24 := bytes[6] + (bytes[7] << 8) + (bytes[8] << 16)

    x-i21 := sign-extend_ x24 SIGNED-21-BIT_
    y-i21 := sign-extend_ y24 SIGNED-21-BIT_
    z-i21 := sign-extend_ z24 SIGNED-21-BIT_

    x := x-i21 * lsb-x_
    y := y-i21 * lsb-y_
    z := z-i21 * lsb-z_

    return Point3f x y z

  read-mag-compensated bytes/ByteArray=read-raw-data -> Point3f:
    x24 := bytes[0] + (bytes[1] << 8) + (bytes[2] << 16)
    y24 := bytes[3] + (bytes[4] << 8) + (bytes[5] << 16)
    z24 := bytes[6] + (bytes[7] << 8) + (bytes[8] << 16)

    x-i21 := sign-extend_ x24 SIGNED-21-BIT_
    y-i21 := sign-extend_ y24 SIGNED-21-BIT_
    z-i21 := sign-extend_ z24 SIGNED-21-BIT_

    x := x-i21 * lsb-x_
    y := y-i21 * lsb-y_
    z := z-i21 * lsb-z_
    t := (read-temp bytes)

    otp-output := otp-data_.apply-compensation x y z t
    return Point3f otp-output[0] otp-output[1] otp-output[2]

  read-sensortime bytes/ByteArray=read-raw-data -> int:
    st := bytes[12] + (bytes[13] << 8) + (bytes[14] << 16)
    st-i24 := sign-extend_ st SIGNED-24-BIT_
    return st-i24

  read-temp bytes/ByteArray=read-raw-data -> float:
    t24 := bytes[9] + (bytes[10] << 8) + (bytes[11] << 16)
    t-i24 := sign-extend_ t24 SIGNED-24-BIT_
    temp := t-i24 * lsb-t_
    // Correct for temperature (see Bosch reference C implementation).
    if temp > 0.0:
      temp = temp - 25.49
    else if temp < 0.0:
      temp = temp + 25.49
    return temp

  /**
  Gets OTP data into the class variable.

  For the BMM350, all the math of conversion, calibration, etc, must be done in
    software.Puts OTP compensation data into a class variable (instance of a
    parser).  This data must be obtained after boot and before use.
  */
  get-otp-data_ -> none:
    otp-cmd  := OTP-CMD-DIR-READ_ << OTP-CMD-CMD-MASK_.count-trailing-zeros
    OTP-DATA-LENGTH.repeat: | otp-word-index |
      otp-addr := otp-word-index & OTP-CMD-WORD-MASK_
      otp-pkg  := (otp-cmd | otp-addr) & 0xFF
      write-register_ REG-OTP-CMD_ otp-pkg
      if not is-otp-cmd-done_:
        throw "OTP Error"

      otp-lsb := read-register_ REG-OTP-DATA-LSB_
      otp-msb := read-register_ REG-OTP-DATA-MSB_

      otp-data_[otp-word-index] = #[otp-lsb, otp-msb]
      /*logger_.debug "otp index gets data" --tags={
        "otp-pkg": bits-grouped_ otp-pkg --group-size=8,
        "index": otp-word-index,
        "data": #[otp-msb, otp-lsb]}
      */

  is-otp-cmd-done_ -> bool:
    raw := read-register_ REG-OTP-STATUS_
    error := (raw & OTP-STATUS-ERR-MASK_) >> OTP-STATUS-ERR-MASK_.count-trailing-zeros
    page := (raw & OTP-STATUS-CURR-PAGE-ADDR_) >> OTP-STATUS-CURR-PAGE-ADDR_.count-trailing-zeros
    done := (raw & OTP-STATUS-CMD-DONE_)
    if error != OTP-STATUS-ERR-NO-ERROR_:
      logger_.error "OTP Error" --tags={"err-code":error, "page":page, "error-text":OTP-STATUS-ERR-LOOKUP_[error]}
    return done == 1

  is-pmu-cmd-illegal_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-CMD-ILLEGAL_) == 1

  is-pmu-pwr-normal_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-PWR-NORMAL_) == 1

  is-pmu-avg-overwrite_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-AVG-OVWERWRITE_) == 1

  is-pmu-odr-overwrite_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-ODR-OVWERWRITE_) == 1

  is-pmu-cmd-busy_ -> bool:
    return (read-register_ REG-PMU-CMD-STATUS-0_ --mask=CMD-STATUS-0-PMU-CMD-BUSY_) == 1

  /**
  Whether a PMU Error has been raised.

  Error clears on read.
  */
  is-pmu-error -> bool:
    return (read-register_ REG-ERR-REG_ --mask=ERR-REG-PMU-CMD-ERR_) == 1

  /**
  Set PMU Mode
  */
  set-mode mode/int=PMU-CMD-MODE-NORMAL_ -> none:
    assert: PMU-CMD-MODE-LOOKUP_.contains mode
    write-register_ REG-PMU-CMD_ mode --mask=PMU-CMD-MODE-MASK_
    while is-pmu-cmd-busy_:
      sleep --ms=10
    write-register_ REG-PMU-CMD_ PMU-CMD-MODE-UPD-OAE_ --mask=PMU-CMD-MODE-MASK_
    while is-pmu-cmd-busy_:
      sleep --ms=10
    logger_.info "mode configured" --tags={"mode":PMU-CMD-MODE-LOOKUP_[mode]} //, "mode illegal": is-pmu-cmd-illegal_}

  /**
  Get PMU Mode

  Gets the value by looking at the 'effective' mode, not modes' own
    configuration register.
  */
  get-odr -> int:
    odr := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-ODR-EFFECTIVE_
    logger_.debug "effective odr" --tags={"odr":"$(%02x odr)","text":AGGR-ODR-LOOKUP_[odr]}
    return odr

  get-avg -> int:
    avg := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-AVG-EFFECTIVE_
    logger_.debug "effective avg" --tags={"avg":"$(%02x avg)","text":AGGR-AVG-LOOKUP_[avg]}
    return avg

  set-odr odr/int=AGGR-ODR-50HZ_ -> none:
    assert: AGGR-ODR-LOOKUP_.contains odr
    write-register_ REG-PMU-CMD-AGGR-SET_ odr --mask=PMU-CMD-AGGR-ODR-MASK_
    clamp-avg_ odr
    update-aggr-values_
    effective := get-odr

  set-avg avg/int=AGGR-AVG-NONE_ -> none:
    assert: AGGR-AVG-LOOKUP_.contains avg
    write-register_ REG-PMU-CMD-AGGR-SET_ avg --mask=PMU-CMD-AGGR-AVG-MASK_
    clamp-odr_ avg
    update-aggr-values_
    effective := get-avg

  configure-mag --avg/int=AGGR-AVG-NONE_ --odr/int=AGGR-ODR-50HZ_ -> none:
    set-avg avg
    set-odr odr

  update-aggr-values_ -> none:
    start := Time.monotonic-us
    write-register_ REG-PMU-CMD_ PMU-CMD-MODE-UPD-OAE_ --mask=PMU-CMD-MODE-MASK_
    while is-pmu-cmd-busy_:
      sleep --ms=10
    finish := Time.monotonic-us - start
    //logger_.debug "aggr value update complete" --tags={"us":"$finish"}

  /*
  Bosch clamps averaging at high ODR (e.g., 400 Hz forces no averaging;
    200 Hz max avg 2; 100 Hz max avg 4.
  */
  clamp-avg_ target-odr/int -> none:
    current-avg := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-AVG-EFFECTIVE_
    if ODR-CLAMP-AVG-LOOKUP_.contains target-odr:
      if ODR-CLAMP-AVG-LOOKUP_[target-odr] > current-avg:
        write-register_ REG-PMU-CMD-AGGR-SET_ ODR-CLAMP-AVG-LOOKUP_[target-odr] --mask=PMU-CMD-AGGR-AVG-MASK_
        logger_.debug "aggr avg value clamped due to high ODR" --tags={"odr":AGGR-ODR-LOOKUP_[target-odr],"avg-clamped": AGGR-AVG-LOOKUP_[ODR-CLAMP-AVG-LOOKUP_[target-odr]]}

  /*
  Bosch clamps averaging at high ODR (e.g., 400 Hz forces no averaging;
    200 Hz max avg 2; 100 Hz max avg 4.
  */
  clamp-odr_ target-avg/int -> none:
    current-odr := read-register_ REG-PMU-CMD-STATUS-1_ --mask=CMD-STATUS-1-ODR-EFFECTIVE_
    if AVG-CLAMP-ODR-LOOKUP_.contains target-avg:
      if AVG-CLAMP-ODR-LOOKUP_[target-avg] < current-odr:
        write-register_ REG-PMU-CMD-AGGR-SET_ AVG-CLAMP-ODR-LOOKUP_[target-avg] --mask=PMU-CMD-AGGR-ODR-MASK_
        logger_.debug "aggr odr value clamped due to high avg" --tags={"avg":AGGR-AVG-LOOKUP_[target-avg],"odr-clamped": AGGR-ODR-LOOKUP_[AVG-CLAMP-ODR-LOOKUP_[target-avg]]}

  /**
  Reads and optionally masks/parses register data. (Little-endian.)
  */
  /*
  BMM350 has documented behaviour of writing two (DUMMY-BYTES_) bytes for every
    read.  These should not be written - write-register_ not adjusted.
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

  /**
  Writes register data - either masked or full register writes. (Little-endian.)
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

    // Check mask fits register width:
    if width == WIDTH-8_: assert: (mask & ~0xFF) == 0
    else: assert: (mask & ~0xFFFF) == 0

    // Determine if write is full width:
    full-width := (offset == 0) and ((width == WIDTH-8_ and mask == 0xFF) or (width == 16 and mask == 0xFFFF))

    // For now don't accept negative numbers as masked writes.
    if signed and not full-width:
      throw "masked signed write not supported (encode to field bits first)"

    // Mask must fit within the register width:
    field-mask/int := mask >> offset
    assert: field-mask != 0

    // Check an unsigned write is actually > 0:
    if not signed:
      assert: value >= 0 and value <= field-mask
    else:
      if width == WIDTH-8_: assert: -128 <= value and value <= 127
      else: assert: -32768 <= value and value <= 32767

    // Full-width direct write:
    if full-width:
      if width == WIDTH-8_:
        signed ? reg_.write-i8 register value : reg_.write-u8 register value
      else:
        signed ? reg_.write-i16-le register value : reg_.write-u16-le register value
      return

    // Read Reg for modification:
    old-value/int := (width == WIDTH-8_) ? reg_.read-u8 register : reg_.read-u16-le register
    reg-mask/int := (width == WIDTH-8_) ? 0xFF : 0xFFFF
    new-value/int := (old-value & ~mask) | ((value & field-mask) << offset) & reg-mask

    // Write modified value:
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

    // raw binary
    bin := "$(%b x)"

    // choose target width: at least min-display-bits, then round up to a full group
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

    // left-pad to target width
    bin = bin.pad --left width '0'

    // group left->right
    out := ""
    i := 0
    while i < bin.size:
      if i > 0: out = "$(out)$(sep)"
      j := i + group-size
      if j > bin.size: j = bin.size
      out = "$(out)$(bin[i..j])"
      i = j

    return out

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

  // Apply Compensation - Variant taking point3f.
  apply-compensation point/Point3f t/float -> List:
    return apply-compensation point.x point.y point.z t

  // Apply Compensation.
  apply-compensation x/float y/float z/float t/float -> List:
    // Temp compensation.
    t = (1.0 + sens-t) * t + offset-t
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

  stringify -> string:
    out-list := List Bmm350.OTP-DATA-LENGTH
    payload_.size.repeat: | index |
      out-list[index] = "0x$(%04x payload_[index])"
    return "[$(out-list.join ", ")]"

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
