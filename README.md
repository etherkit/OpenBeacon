OpenBeacon
==========

A QRSS MEPT.

Homebrewing Your Own
--------------------

If you are homebrewing OpenBeacon and are burning a copy of the firmware .hex file on your own ATtiny85, then be sure to use the following fuse settings in order for the microcontroller to work correctly:

Low: 0xE1<br/>
High: 0x5D<br/>
Ext: 0xFF

If you are using a \*nix machine, I've included a handy script to burn the firmware and set the fuses called `burn_firmware.sh` in the firmware/Release folder.
