# Flight-Computer-Firmware

This is the current latest version of the flight computer, currently only build for the Teensy microcontrollers.


# TODO

### Easiest:

- [ ] I2C with LTC2990
- [ ] Morse code buzzer on boot (beep boop codes)
- [ ] I think my Gyro driver for the BMI088 might need to be tested more?

### Medium:

- [ ] SPI with NEOGPS (Can use the old flight computer as a test)
- [ ] Decode or get a library for GMEA decoding: we only want LAT LON and ALT from gps data
- [ ] Ground station update/reface [Ground station revamp](https://github.com/University-at-Buffalo-SEDS/Flight-Computer-Ground-Station)

### Very difficult:
*requires a decent amount of reading compared to the others*

- [ ] CAN BUS
- [ ] APRS
    - [ ] (AX.25 <-> AFSK <-> ~~DMA~~ <-> ~~DAC~~)

### Optional?

- [ ] Either desktop program to get data from flight computer via usb serial OR
   Emulate a mass storage device and show the flight data as files. This one would be cool and requires no software to be on another person's computer to get the data. But would require figuring out how USB mass storage protocol works.

### Testing

- [ ] Bandwidth test of RFD900x (all the data + payload data if that happens, this should be pretty easy, just a day or two)
- [ ] Bandwidth test of CAN bus


