#!/usr/bin/python3

# This script is a utility that pulls images off of an unformatted SD card and turns them into a
# video. Images should be once every 0x3000 bytes at an offset of 0x200, e.g.
#     0x0000_0200, 0x0000_3200, 0x0000_6200, ...
#
# It also pulls MLX 'images' and renders them side-by-side with the hm01b0 video.
#     MLX calibraiton data:    0x6_0000_0200 - 0x6_0000_09ff
#     Metadata 0               0x6_0000_0a00 - 0x6_0000_0bff
#     Frame 0                  0x6_0000_0c00 - 0x6_0000_13ff
#     Metadata 1               0x6_0000_1400 - 0x6_0000_05ff
#     Frame 1                  0x6_0000_1600 - 0x6_0000_1dff
#
# This script assumes that the SD card is connected to /dev/sdb and that the script has read access.

# Example usage:
#     ./scriptname.py <starting frame> <num of frames> <output file>

import colorsys
import cv2
import io
import itertools
import math
import numpy as np
import subprocess
from PIL import Image
import sys
import os

from numpy.lib.stride_tricks import as_strided

import tempfile

helpstr = f"Usage: {sys.argv[0]} <starting frame num> <num of frames> <frame increment> <output file>"

def rip_timestamp(frame):
    searchstr = b'millisecond time:\x00'
    idx = frame.find(searchstr) + len(searchstr)

    # string should be terminated with a space, so we can just convert it directly to an int.
    endidx = idx
    while ((frame[endidx] >= 0x30) and (frame[endidx] <= 0x39)):
        endidx += 1
    return int(frame[idx:endidx])

def rip_mlx_frame(frame, mlx90640_eeprom):
    """
    Given a byte array containing MLX frame metadata and an MLX frame (should be 5 * 512 = 2560B),
    this function uses the mlx_rawdata_decoder command line tool to convert it into a 32x24 numpy
    array of floats, which correspond to temperatures in Celsius.

    Returns: tuple containing timestamp and a 2-D numpy array containing temperatures.
    """

    # get timestamp
    timestamp = rip_timestamp(frame)

    #with tempfile.NamedTemporaryFile(delete=False) as frame_file, tempfile.NamedTemporaryFile(delete=False) as calib_file:
    with open("./frame.bin", "wb") as frame_file, open("./calib.bin", "wb") as calib_file:
        calib_file.write(mlx90640_eeprom)
        frame_file.write(frame[512:-1])
        calib_file.flush()
        frame_file.flush()

        # run command line tool and get results
        toolname = "./mlx_rawdata_decoder/mlx_rawdata_decoder"
        result = subprocess.run([toolname, '-i', frame_file.name, '-c', calib_file.name], stdout=subprocess.PIPE).stdout.decode("utf-8")
        lines = result.split("\n")
        temperatures = None
        for l in lines:
            try:
                strs = [s for s in l.strip().split(',') if (len(s) > 0)]
                temperatures = np.array([float(s) for s in strs[1:]]).reshape((24, 32))

                # smear checkerboard for fast fix
                """
                for i in range(0, len(temperatures), 2):
                    if (temperatures[i] == 0):
                        temperatures[i] = temperatures[i + 1]
                    else:
                        temperatures[i + 1] = temperatures[i]
                """

            except Exception as ex:
                pass
                #print(ex)

            if (temperatures is not None):
                break

    return timestamp, temperatures

def smooth_mlx_frame(frame):
    """
    The MLX checkerboard pattern is best overcome by averaging the 4 surrounding samples
    """
    OFFSETS = [[-1, 0], [0, 1], [1, 0], [0, -1]]
    for y in range(len(frame)):
        for x in range(len(frame[y])):
            s = 0
            numel = 0
            if (frame[y][x] == 0):
                for o in OFFSETS:
                    try:
                        s += frame[y + o[0]][x + o[1]]
                        numel += 1
                    except:
                        pass

                frame[y][x] = s / numel

    return frame

def colorize_mlx_frame(frame):
    """
    Given an MLX frame as a 2D numpy array of temperature floats, this function converts it to a
    3D numpy array of RGB values (e.g. [[[0, 0, 255], [0, 255, 0]], [[0, 255, 0], [0, 0, 255]]])
    """
    minval = 100
    for v in frame.flatten():
        if (v < minval): minval = v

    r = frame.max() - minval
    if (r < 10):
        r = 10

    frame_rgb = []
    for l in frame:
        line_rgb = []
        for val in l:
            hue = (val - minval) / r

            if (math.isnan(hue)):
                hue = 0

            # map [0, 1) to (1, 2/3]
            #print(hue)
            hue /= -3
            hue += 1    # (1 / 3) + (2 / 3)
            rgbval = [int(c * 255) for c in colorsys.hsv_to_rgb(hue, 1, 1)]
            line_rgb.append(rgbval)
        frame_rgb.append(line_rgb)

    return np.asarray(frame_rgb, dtype=np.uint8)

def tile_array(a, b0, b1):
    """
    taken from
    https://stackoverflow.com/questions/32846846/quick-way-to-upsample-numpy-array-by-nearest-neighbor-tiling
    """
    r, c = a.shape                                    # number of rows/columns
    rs, cs = a.strides                                # row/column strides
    x = as_strided(a, (r, b0, c, b1), (rs, 0, cs, 0)) # view a as larger 4D array
    return x.reshape(r*b0, c*b1)                      # create new 2D array

def rip_jpeg_frame(frame, jpeg_header):
    """
    Given a byte array containing jpeg frame metadata and a jpeg (should be a total of
    24 * 512 = 12288 Bytes), this function gets out the jpeg image.

    Returns: a tuple containing the timestamp of the jpeg frame metadata and a BGR numpy array.
    """
    # get timestamp
    timestamp = rip_timestamp(frame)

    # convert image
    pil_im = Image.open(io.BytesIO(jpeg_header + frame[512:]))
    frame = cv2.cvtColor(np.array(pil_im), cv2.COLOR_GRAY2BGR)

    return timestamp, frame

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

    print("Make sure to specify the file names directly in the script!!")
    JPEG_FRAME_LEN = 32 * 512
    JPEG_FRAME_OFFSET = 512

    MLX_FRAME_LEN = 512 * 5
    #MLX_CALIB_START = 512 * 0x03000000
    #MLX_CALIB_START = 512 * 0x00c00000
    MLX_CALIB_START = 0
    MLX_DATA_START = MLX_CALIB_START + MLX_FRAME_LEN

    #with open("/dev/mmcblk0", "rb") as jpeg_file_pointer, open("/dev/mmcblk0", "rb") as mlx_file_pointer:
    with open("../datalogs/2021-06-02_16-40_outdoor-indoor/jpeg-data", "rb") as jpeg_file_pointer, open("../datalogs/2021-06-02_16-40_outdoor-indoor/mlx-data", "rb") as mlx_file_pointer:
        print("seeking... ")

        # Get mlx eeprom
        mlx_file_pointer.seek(MLX_CALIB_START + 512)
        mlx90640_eeprom = mlx_file_pointer.read(512 * 4)

        jpeg_file_pointer.seek(starting_frame * JPEG_FRAME_LEN)

        print("converting... ")

        # Get an initial frame so we can get the dimensions
        block = jpeg_file_pointer.read(JPEG_FRAME_LEN)
        pil_im = Image.open(io.BytesIO(jpeg_header + block[1024:-1]))

        frame = cv2.cvtColor(np.array(pil_im), cv2.COLOR_GRAY2BGR)
        height, width, layers = frame.shape

        # setup video
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        video = cv2.VideoWriter(sys.argv[4], fourcc, 19, (width * 2, height))

        # rip jpeg frames to array of (timestamp, image) tuples
        jpeg_frames = []
        for i in range(0, num_frames, increment):
            print(f"reading jpeg frame {i + 1} / {num_frames}\n", end='')

            # read JPEG image
            jpeg_block = jpeg_file_pointer.read(JPEG_FRAME_LEN)
            jpeg_timestamp, jpeg_image = rip_jpeg_frame(jpeg_block, jpeg_header)
            jpeg_frames.append((jpeg_timestamp, np.rot90(jpeg_image, 2)))

        # need to scrub through the mlx frames until we find the one with the timestamp right before
        # the jpeg frame we want to start reading from. After this loop, the mlx file pointer will
        # be at the right spot to start reading mlx frames from.
        MAX_MLX_FRAME_ATTEMPT = 3125000
        print("seeking for first mlx frame...")
        for i in range(MAX_MLX_FRAME_ATTEMPT):
            mlx_block = mlx_file_pointer.read(MLX_FRAME_LEN)
            timestamp = rip_timestamp(mlx_block)
            print(f"timestamp: {timestamp}")
            if (timestamp >= jpeg_frames[0][0]):
                print(f"mlx frame {i} at timestamp {timestamp} overlaps with jpeg frame at timestamp {jpeg_frames[0][0]}")
                mlx_file_pointer.seek(-MLX_FRAME_LEN, os.SEEK_CUR)
                break

        # Read and decode mlx frames
        mlx_frames = []
        timestamp_prev = 0
        while (timestamp_prev <= jpeg_frames[-1][0]):
            print(f"decoding mlx frame {len(mlx_frames) + 1}... ", end='')
            mlx_block = mlx_file_pointer.read(MLX_FRAME_LEN)
            mlx_timestamp, mlx_image = rip_mlx_frame(mlx_block, mlx90640_eeprom)
            print(f"timestamp = {mlx_timestamp}, dt = {mlx_timestamp - timestamp_prev}\n", end='')
            mlx_image = np.flip(mlx_image, 0)
            mlx_image = smooth_mlx_frame(mlx_image)
            mlx_image = tile_array(mlx_image, 10, 10)
            mlx_image = colorize_mlx_frame(mlx_image)

            mlx_frames.append((mlx_timestamp, np.rot90(mlx_image, 2)))
            timestamp_prev = mlx_timestamp
        print("")

        for i in range(0, num_frames, increment):
            # linear search through mlx frames to find one closest
            mlx_frame = mlx_frames[0]
            for f in mlx_frames[1:]:
                if (abs(f[0] - jpeg_frames[i][0]) <
                    abs(mlx_frame[0] - jpeg_frames[i][0])):
                    mlx_frame = f

            # write video
            concatenated_frame = np.concatenate((jpeg_frames[i][1], mlx_frame[1]), axis=1)
            video.write(concatenated_frame)

        video.release()
