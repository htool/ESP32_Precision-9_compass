# ESP32_Precision-9_compass
MPU9250 and ESP32 emulation of Precision-9 compass

The aim is to use standard config screen in the B&G Zeus/Vulcan/Triton(2) to configure offsets and do calibration.

Heavily borrowed code from here:
 https://www.instructables.com/Quaternion-Compass/

As first use, TASK needs to be set to 1 for compass (figure 8) calibration.
Then TASK 2 for gyro/accel calibration. This is to allow sensor calibration to happen at home.
The calibration offsets are stored in EEPROM so they don't have to be done again when mounted in the boat.

Afterwards leave in TASK 5.

What's working:
 - Recognised as B&G Precision-9.
 - Heading send out as Vessel Heading PGN 127250 as Magnetic at 10Hz.
 - Magnetic variation PGN is picked up and then Vessel Heading PGN 127250 is send out as True as well.
 - Heel and Trim are send out as Attitude PGN 127257 at 20Hz.
 - Rate of turn is send out as PGN 127251 at 10Hz.
 - MFD configuration of heading, trim and heel offset.
 - Offsets are stored in EEPROM when set.

What's half working:
 - Deviation calibration stop/cancel from MFD. The stop/cancel signal works, but the 'finished' signal is still missing as well as the actual routing. This routine is the 390 deg circle that allows building a boat specific deviation table. I think Fourier transformation could help here. 390 degrees is needed to have a full wave.
 - Auto calibration mode is recognised, but not used yet.

What's not working
 - Heave isn't measured yet (anyone an idea) and thus not sent.
