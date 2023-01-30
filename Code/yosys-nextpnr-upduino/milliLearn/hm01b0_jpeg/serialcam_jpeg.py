#!/usr/bin/env python3

import pygame
import numpy as np
import serial
import io
from PIL import Image

import re
[m.start() for m in re.finditer('test', 'test test test test')]
#[0, 5, 10, 15]

import time

width = 320
height = 240

vstretch = 2
hstretch = 2

# pygame.init()
# pygame.display.set_caption(' ')
# screen = pygame.display.set_mode((width * hstretch, height * vstretch))
# pygame.display.flip()

# ser = serial.Serial('/dev/ttyUSB0', 460800 * 2, timeout=0, parity=serial.PARITY_EVEN)
ser = serial.Serial('/dev/ttyUSB0', 144230*104*100, timeout=0, parity=serial.PARITY_NONE)
running = True
image = [None] * (width * height)
imagestr = b""

millis_prev = int(round(time.time() * 1000))

jpeg_header = b""
with open("./jpeg_header.bin", mode="rb") as f:
    jpeg_header = f.read()

imgarr = []
while running:
    # Read serial data
    # imagestr = bytes()
    # imgarr = []
    while ser.in_waiting:  # Or: while ser.inWaiting():
        foo = ser.read()
        imagestr += foo
        #print(len(imagestr))
        imgarr.append(bytes(foo))
        # print(foo)
        # if (imagestr[-2:] == b"\xff\xd9"):
        #     break
        # print(len(imgarr))
        if(len(imgarr) % 1000 ==0):
            print("1k")
        if(len(imgarr) >= 324*244):
            input("Done")
            break
        

    #delimiters = [m.start() for m in re.finditer(b"\xff\xd9", imagestr)]
    if (imagestr[-2:] == b"\xff\xd9" or len(imagestr) == 324*244):
        try:
            # print(imagestr)

            # add header info
            #mostrecent = imagestr[delimiters[-2]+2:delimiters[-1]+2]
            mostrecent = imagestr
            # print(str(len(jpeg_header + mostrecent)) + " bytes")
            # image = pygame.image.load(io.BytesIO(jpeg_header + mostrecent))
            # i = Image.fromstring('RGB', (324, 244), mostrecent, 'raw', 'BGRX')
            binary_stream = io.BytesIO(bytes(mostrecent))
            im = Image.open(StringIO(imagestr))
            # Open the image using PIL
            # im = Image.open(binary_stream)
            # im = Image.fromarray(imgarr, "RGB")
            # Display the image
            im.show()
            input()


            # image = pygame.image.load(io.BytesIO(mostrecent))
            # image = pygame.transform.scale(image, (width * hstretch, height * vstretch))
            # screen.blit(image, (0, 0))
            # pygame.display.update()

            millis = int(round(time.time() * 1000))
            print(millis - millis_prev)
            millis_prev = millis

        except OSError as e:
            print(e)
            print(' '.join('{:02x}'.format(x) for x in imagestr[0:127]))

        imagestr = b""

    # # check if x was pressed to close window.
    # for event in pygame.event.get():
    #     if event.type == pygame.QUIT:
    #         running = False


# pygame.quit()
