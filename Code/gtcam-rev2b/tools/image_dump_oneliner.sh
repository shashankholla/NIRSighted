cp jpeg_header.bin frame100000.jpeg && sudo dd if=/dev/sdb bs=1 count=$((10000)) skip=$((0x3000*100000 + 0x200)) >> frame100000.jpeg;
