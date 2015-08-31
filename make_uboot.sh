#!/bin/sh
make
nandbootec u-boot.bin.ec /opt/NSIH/DTK.700.350.175.087-192.NSIH.txt ./u-boot.bin 80100000 80100000
scp u-boot.bin.ec u-boot.bin falbb@192.168.2.130:/share/work/nxp2120/7week

