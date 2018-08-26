#!/bin/bash

avrdude -c usbtiny -p t85 -U flash:w:OpenBeacon.hex
avrdude -c usbtiny -p t85 -V -U lfuse:w:0xe1:m -U hfuse:w:0x5d:m -U efuse:w:0xff:m
