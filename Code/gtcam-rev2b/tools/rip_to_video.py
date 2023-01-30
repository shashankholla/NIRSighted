#!/usr/bin/python3

# This script is a utility that pulls images off of an unformatted SD card and turns them into a
# video. Images should be once every 0x3000 bytes at an offset of 0x200, e.g.
#     0x000_0200, 0x0000_3200, 0x0000_6200, ...
#
# This script assumes that the SD card is connected to /dev/sdb and that the script has read access.

# Example usage:
#     ./scriptname.py <starting frame> <num of frames> <output file>

import cv2
import io
import numpy as np
from PIL import Image
import sys
import os

helpstr = f"Usage: {sys.argv[0]} <starting frame num> <num of frames> <frame increment> <output file>"

def rip_size(frame):
    searchstr = b'bytes:\x00'
    idx = frame.find(searchstr) + len(searchstr)

    # string should be terminated with a space, so we can just convert it directly to an int.
    endidx = idx
    while ((frame[endidx] >= 0x30) and (frame[endidx] <= 0x39)):
        endidx += 1
    return int(frame[idx:endidx])

if __name__ == "__main__":
    jpeg_header = b""
    with open("./jpeg_header.bin", mode="rb") as f:
        jpeg_header = f.read()

    try:
        starting_frame = int(sys.argv[1])
        num_frames = int(sys.argv[2])
        increment = int(sys.argv[3])
    except:
        print("Starting frame num and num of frames need to be positive integers.")
        print(helpstr)
        sys.exit(1)

    FRAME_LEN = 512 * 32
    FRAME_OFFSET = 512

    print(sys.argv[4])

    with open("/dev/mmcblk0", "rb") as sdcard:
        print("seeking... ")
        sdcard.seek(starting_frame * FRAME_LEN)

        print("converting... ")

        # Get an initial frame so we can get the dimensions
        block = sdcard.read(FRAME_LEN)
        pil_im = Image.open(io.BytesIO(jpeg_header + block[512:-1]))

        frame = cv2.cvtColor(np.array(pil_im), cv2.COLOR_GRAY2BGR)
        height, width, layers = frame.shape

        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        video = cv2.VideoWriter(sys.argv[4], fourcc, 15, (width, height))

        for i in range(0, num_frames, increment):
            block = sdcard.read(FRAME_LEN)
            print(f"on frame {i + 1} / {num_frames}. frame size: {rip_size(block[0:512])}\n", end='')
            pil_im = Image.open(io.BytesIO(jpeg_header + block[512:-1]))
            try:
                frame = cv2.cvtColor(np.array(pil_im), cv2.COLOR_GRAY2BGR)
            except:
                frame = cv2.cvtColor(np.zeros((320, 240), dtype=np.uint8), cv2.COLOR_GRAY2BGR)

            video.write(frame)

            sdcard.seek(((increment - 1) * FRAME_LEN), 1)

        print()
        video.release()
