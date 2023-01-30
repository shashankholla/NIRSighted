#!/bin/bash
echo $1
f=frame${1}.jpeg
echo "saving to file " $f
cp jpeg_header.bin $f
dd if=/dev/mmcblk0 bs=1 count=$((16536)) skip=$((0x4000*$1 + 0x200)) >> $f;
eog $f
sleep 0.01
rm $f
