# read text in block right before mlx calibration data
dd skip=50331648  if=/dev/sdb of=sometext.bin bs=512 count=1 2>/dev/null

# read mlx calibration data
dd skip=50331649  if=/dev/sdb of=calib.bin bs=512 count=4 2>/dev/null

# hexdump SD card at big offset using dd
dd if=/dev/mmcblk0 bs=512 count=128 skip=$((0x00b8000)) 2>/dev/null | hexdump -C

# convert avi to gif
ffmpeg -ss <start_time> -t <length_to_convert> -f gif -i infile.avi output.gif


# for dumping...
# jpeg (replace '0' with the number of seconds)
dd if=/dev/mmcblk0 of=jpeg-data bs=512 count=$((32 * 20 * 0))

# mlx (replace '0' with the number of seconds)
dd if=/dev/mmcblk0 of=mlx-data bs=512 skip=$((0x00c00000)) count=$((5 * 16 * 0))
