# -*- coding: UTF-8 -*-
import time
import argparse
from rpi_ws281x import ws, Color, Adafruit_NeoPixel
from collections import OrderedDict
import socket
import random
import signal
import multiprocessing
import serial
import os 

# LED strip configuration:
FRONT_SCREEN_PIN = 12
FRONT_LAMP_PIN = 19 
BACK_SCREEN_PIN = 18
BACK_LAMP_PIN = 13 

LED_1_COUNT = 512        # Number of LED pixels.
LED_1_PIN = FRONT_SCREEN_PIN   # GPIO pin connected to the pixels (must support PWM! GPIO 13 and 18 on RPi 3).
LED_1_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_1_DMA = 10          # DMA channel to use for generating signal (Between 1 and 14)
LED_1_BRIGHTNESS = 20  # Set to 0 for darkest and 255 for brightest
LED_1_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_1_CHANNEL = 0       # 0 or 1
LED_1_STRIP = ws.WS2811_STRIP_GRB

LED_2_COUNT = 140        # Number of LED pixels.
LED_2_PIN = FRONT_LAMP_PIN   # GPIO pin connected to the pixels (must support PWM! GPIO 13 or 18 on RPi 3).
LED_2_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_2_DMA = 10          # DMA channel to use for generating signal (Between 1 and 14)
LED_2_BRIGHTNESS = 20  # Set to 0 for darkest and 255 for brightest
LED_2_INVERT = False    # True to invert the signal (when using NPN transistor level shift)
LED_2_CHANNEL = 1       # 0 or 1
LED_2_STRIP = ws.WS2811_STRIP_GRB

LED_3_PIN = BACK_SCREEN_PIN
LED_4_PIN = BACK_LAMP_PIN

def _signal_handler(signum, frame):
  server_socket.close()
  print('Received exit signal: ', signum)
  exit(0)

def get_host_ip():
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    ip = s.getsockname()[0]
  finally:
    s.close()
  return ip

BUFSIZE = 1024
host_ip = get_host_ip()
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
server_socket.bind(("0.0.0.0",9090))
send_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

def multiColorWipe(strip, color, dis_str, wait_ms=3,bias=0,equal=False):
  """Wipe color across multiple LED strips a pixel at a time."""
  if dis_str not in ["HELLO","CAUTION","<<<<",">>>>"]:
    for i in range(strip.numPixels()):
      if i < bias: continue
      strip.setPixelColor(i, color)
      # strip.show()
      # time.sleep(wait_ms / 1000.0)
    strip.show()
  elif dis_str == "HELLO":      
    pixel_num = strip.numPixels()-bias
    for i in range(pixel_num/2):
      displays = range(bias,bias+i+1) + range(pixel_num-i-1+bias,pixel_num+bias)
      for sub_i in range(bias,pixel_num+bias):
        if sub_i in displays:
          strip.setPixelColor(sub_i, color)
        else:
          strip.setPixelColor(sub_i, Color(0, 0, 0))
      strip.show()
      time.sleep(0.02)

  elif dis_str == "CAUTION":
    pixel_num = strip.numPixels()-bias
    for index in range(10):
      for i in range(pixel_num):
        if index%2 != 0:
          strip.setPixelColor(i+bias, color)
        else:
          strip.setPixelColor(i+bias, Color(0, 0, 0))
      strip.show()
      time.sleep(0.4)
  elif (dis_str == "<<<<" and host_ip == "192.168.1.66") or (dis_str == ">>>>" and host_ip == "192.168.1.65"):
    blackout(strip)
    for i in range(strip.numPixels()):
      if i < bias: continue
      strip.setPixelColor(i, color)
      strip.show()
      time.sleep(0.01)
  elif (dis_str == ">>>>" and host_ip == "192.168.1.66") or (dis_str == "<<<<" and host_ip == "192.168.1.65"):
    blackout(strip)
    for i in range(strip.numPixels(),0,-1):
      if i < bias: continue
      strip.setPixelColor(i, color)
      strip.show()
      time.sleep(0.01)

  if dis_str == "GOODBYE":
    pixel_num = strip.numPixels()-bias
    for i in range(pixel_num/2):
      displays = range(bias+pixel_num/2-i-1,bias+pixel_num/2) + range(bias+pixel_num/2,bias+pixel_num/2+i+1)
      for sub_i in range(bias,pixel_num+bias):
        if sub_i in displays:
          strip.setPixelColor(sub_i, Color(0, 0, 0))
        else:
          strip.setPixelColor(sub_i, color)
      strip.show()
      time.sleep(0.02)

def blackout(strip,dis_seq=[]):
  if not dis_seq:
    for i in range(strip.numPixels()):
      strip.setPixelColor(i, Color(0, 0, 0))
  else:
    for i in dis_seq:
      strip.setPixelColor(i, Color(0, 0, 0))
  strip.show()    

# ascii_map={"0":[0x3E,0x51,0x49,0x45,0x3E,0x00],"1":[0x00,0x42,0x7F,0x40,0x00,0x00],
#            "2":[0x42,0x61,0x51,0x49,0x46,0x00],"3":[0x21,0x41,0x45,0x4B,0x31,0x00],
#            "4":[0x18,0x14,0x12,0x7F,0x10,0x00],"5":[0x27,0x45,0x45,0x45,0x39,0x00],
#            "6":[0x3C,0x4A,0x49,0x49,0x30,0x00],"7":[0x01,0x71,0x09,0x05,0x03,0x00],
#            "8":[0x36,0x49,0x49,0x49,0x36,0x00],"9":[0x06,0x49,0x49,0x29,0x1E,0x00],
#            "A":[0x7E,0x11,0x11,0x11,0x7E,0x00],"B":[0x7F,0x49,0x49,0x49,0x36,0x00],
#            "C":[0x3E,0x41,0x41,0x41,0x22,0x00],"D":[0x7F,0x41,0x41,0x22,0x1C,0x00],
#            "E":[0x7F,0x49,0x49,0x49,0x41,0x00],"F":[0x7F,0x09,0x09,0x09,0x01,0x00],
#            "G":[0x3E,0x41,0x49,0x49,0x7A,0x00],"H":[0x7F,0x08,0x08,0x08,0x7F,0x00],
#            "I":[0x00,0x41,0x7F,0x41,0x00,0x00],"J":[0x20,0x40,0x41,0x3F,0x01,0x00],
#            "K":[0x7F,0x08,0x14,0x22,0x41,0x00],"L":[0x7F,0x40,0x40,0x40,0x40,0x00],
#            "M":[0x7F,0x02,0x0C,0x02,0x7F,0x00],"N":[0x7F,0x04,0x08,0x10,0x7F,0x00],
#            "O":[0x3E,0x41,0x41,0x41,0x3E,0x00],"P":[0x7F,0x09,0x09,0x09,0x06,0x00],
#            "Q":[0x3E,0x41,0x51,0x21,0x5E,0x00],"R":[0x7F,0x09,0x19,0x29,0x46,0x00],
#            "S":[0x46,0x49,0x49,0x49,0x31,0x00],"T":[0x01,0x01,0x7F,0x01,0x01,0x00],
#            "U":[0x3F,0x40,0x40,0x40,0x3F,0x00],"V":[0x1F,0x20,0x40,0x20,0x1F,0x00],
#            "W":[0x3F,0x40,0x38,0x40,0x3F,0x00],"X":[0x63,0x14,0x08,0x14,0x63,0x00],
#            "Y":[0x03,0x04,0x78,0x04,0x03,0x00],"Z":[0x61,0x51,0x49,0x45,0x43,0x00],
#            ":":[0x00,0x36,0x36,0x00,0x00,0x00],"<":[0x08,0x14,0x22,0x41,0x00,0x00],
#            ">":[0x41,0x22,0x14,0x08,0x00,0x00],"a":[0x20,0x54,0x54,0x54,0x78,0x00],
#            "b":[0x7F,0x48,0x44,0x44,0x38,0x00],"c":[0x38,0x44,0x44,0x44,0x20,0x00],
#            "d":[0x38,0x44,0x44,0x48,0x7F,0x00],"e":[0x38,0x54,0x54,0x54,0x18,0x00],
#            "f":[0x08,0x7E,0x09,0x01,0x02,0x00],"g":[0x08,0x54,0x54,0x54,0x3C,0x00],
#            "h":[0x7F,0x08,0x04,0x04,0x78,0x00],"i":[0x00,0x48,0x7D,0x40,0x00,0x00],
#            "j":[0x20,0x40,0x44,0x3D,0x00,0x00],"k":[0x7F,0x10,0x28,0x44,0x00,0x00],
#            "l":[0x00,0x41,0x7F,0x40,0x00,0x00],"m":[0x7C,0x04,0x78,0x04,0x78,0x00],
#            "n":[0x7C,0x08,0x04,0x04,0x78,0x00],"o":[0x38,0x44,0x44,0x44,0x38,0x00],
#            "p":[0x7C,0x14,0x14,0x14,0x08,0x00],"q":[0x08,0x14,0x14,0x18,0x7C,0x00],
#            "r":[0x7C,0x08,0x04,0x04,0x08,0x00],"s":[0x48,0x54,0x54,0x54,0x20,0x00],
#            "t":[0x04,0x3F,0x44,0x40,0x20,0x00],"u":[0x3C,0x40,0x40,0x20,0x7C,0x00],
#            "v":[0x1C,0x20,0x40,0x20,0x1C,0x00],"w":[0x3C,0x40,0x30,0x40,0x3C,0x00],
#            "x":[0x44,0x28,0x10,0x28,0x44,0x00],"y":[0x0C,0x50,0x50,0x50,0x3C,0x00],
#            "z":[0x44,0x64,0x54,0x4C,0x44,0x00]," ":[0x00,0x00,0x00,0x00,0x00,0x00],
#            "#":[0x14,0x7F,0x14,0x7F,0x14,0x00],"$":[0x24,0x2A,0x7F,0x2A,0x12,0x00],
#            "&":[0x36,0x49,0x55,0x22,0x50,0x00],")":[0x00,0x41,0x22,0x1C,0x00,0x00],
#            ".":[0x00,0x60,0x60,0x00,0x00,0x00],"?":[0x02,0x01,0x51,0x09,0x06,0x00],
#            "@":[0x32,0x49,0x79,0x41,0x3E,0x00],";":[0x00,0x56,0x36,0x00,0x00,0x00],
#            "!":[0x00,0x00,0x4F,0x00,0x00,0x00],"(":[0x00,0x1C,0x22,0x41,0x00,0x00],
#            ",":[0x00,0x50,0x30,0x00,0x00,0x00],
#            "+":[0x1E,0x30,0x78,0x60,0x00,0x00],"-":[0x00,0x00,0x60,0x78,0x30,0x1E],
#            "*":[0x30,0x38,0x1C,0x14,0x04,0x00],"/":[0x00,0x04,0x14,0x1C,0x38,0x30],
#            "%":[0x3C,0x4A,0x99,0xF9,0xF9,0x99,0x4A,0x3C],
#            "=":[0x3C,0x42,0x81,0x81,0x81,0xBC,0x42,0xBD,0x81,0x81,0xBC,0x42,0xBD,
#                 0x81,0x81,0xBC,0x42,0xBD,0x81,0x81,0x81,0x42,0x3C],
# }

ascii_map={"0":[0x3E,0x51,0x49,0x45,0x3E,0x00],"1":[0x00,0x42,0x7F,0x40,0x00,0x00],
           "2":[0x42,0x61,0x51,0x49,0x46,0x00],"3":[0x21,0x41,0x45,0x4B,0x31,0x00],
           "4":[0x18,0x14,0x12,0x7F,0x10,0x00],"5":[0x27,0x45,0x45,0x45,0x39,0x00],
           "6":[0x3C,0x4A,0x49,0x49,0x30,0x00],"7":[0x01,0x71,0x09,0x05,0x03,0x00],
           "8":[0x36,0x49,0x49,0x49,0x36,0x00],"9":[0x06,0x49,0x49,0x29,0x1E,0x00],
           "A":[0x7E,0x11,0x11,0x11,0x7E,0x00],"B":[0x7F,0x49,0x49,0x49,0x36,0x00],
           "C":[0x3E,0x41,0x41,0x41,0x22,0x00],"D":[0x7F,0x41,0x41,0x22,0x1C,0x00],
           "E":[0x7F,0x49,0x49,0x49,0x41,0x00],"F":[0x7F,0x09,0x09,0x09,0x01,0x00],
           "G":[0x3E,0x41,0x49,0x49,0x7A,0x00],"H":[0x7F,0x08,0x08,0x08,0x7F,0x00],
           "I":[0x00,0x41,0x7F,0x41,0x00,0x00],"J":[0x20,0x40,0x41,0x3F,0x01,0x00],
           "K":[0x7F,0x08,0x14,0x22,0x41,0x00],"L":[0x7F,0x40,0x40,0x40,0x40,0x00],
           "M":[0x7F,0x02,0x0C,0x02,0x7F,0x00],"N":[0x7F,0x04,0x08,0x10,0x7F,0x00],
           "O":[0x3E,0x41,0x41,0x41,0x3E,0x00],"P":[0x7F,0x09,0x09,0x09,0x06,0x00],
           "Q":[0x3E,0x41,0x51,0x21,0x5E,0x00],"R":[0x7F,0x09,0x19,0x29,0x46,0x00],
           "S":[0x46,0x49,0x49,0x49,0x31,0x00],"T":[0x01,0x01,0x7F,0x01,0x01,0x00],
           "U":[0x3F,0x40,0x40,0x40,0x3F,0x00],"V":[0x1F,0x20,0x40,0x20,0x1F,0x00],
           "W":[0x3F,0x40,0x38,0x40,0x3F,0x00],"X":[0x63,0x14,0x08,0x14,0x63,0x00],
           "Y":[0x03,0x04,0x78,0x04,0x03,0x00],"Z":[0x61,0x51,0x49,0x45,0x43,0x00],
           ":":[0x00,0x36,0x36,0x00,0x00,0x00],"<":[0x08,0x1c,0x36,0x63,0x41,0x00],
           ">":[0x41,0x63,0x36,0x1c,0x08,0x00],"a":[0x20,0x54,0x54,0x54,0x78,0x00],
           "b":[0x7F,0x48,0x44,0x44,0x38,0x00],"c":[0x38,0x44,0x44,0x44,0x20,0x00],
           "d":[0x38,0x44,0x44,0x48,0x7F,0x00],"e":[0x38,0x54,0x54,0x54,0x18,0x00],
           "f":[0x08,0x7E,0x09,0x01,0x02,0x00],"g":[0x08,0x54,0x54,0x54,0x3C,0x00],
           "h":[0x7F,0x08,0x04,0x04,0x78,0x00],"i":[0x00,0x48,0x7D,0x40,0x00,0x00],
           "j":[0x20,0x40,0x44,0x3D,0x00,0x00],"k":[0x7F,0x10,0x28,0x44,0x00,0x00],
           "l":[0x00,0x41,0x7F,0x40,0x00,0x00],"m":[0x7C,0x04,0x78,0x04,0x78,0x00],
           "n":[0x7C,0x08,0x04,0x04,0x78,0x00],"o":[0x38,0x44,0x44,0x44,0x38,0x00],
           "p":[0x7C,0x14,0x14,0x14,0x08,0x00],"q":[0x08,0x14,0x14,0x18,0x7C,0x00],
           "r":[0x7C,0x08,0x04,0x04,0x08,0x00],"s":[0x48,0x54,0x54,0x54,0x20,0x00],
           "t":[0x04,0x3F,0x44,0x40,0x20,0x00],"u":[0x3C,0x40,0x40,0x20,0x7C,0x00],
           "v":[0x1C,0x20,0x40,0x20,0x1C,0x00],"w":[0x3C,0x40,0x30,0x40,0x3C,0x00],
           "x":[0x44,0x28,0x10,0x28,0x44,0x00],"y":[0x0C,0x50,0x50,0x50,0x3C,0x00],
           "z":[0x44,0x64,0x54,0x4C,0x44,0x00]," ":[0x00,0x00,0x00,0x00,0x00,0x00],
           "#":[0x14,0x7F,0x14,0x7F,0x14,0x00],"$":[0x24,0x2A,0x7F,0x2A,0x12,0x00],
           "&":[0x36,0x49,0x55,0x22,0x50,0x00],")":[0x00,0x41,0x22,0x1C,0x00,0x00],
           ".":[0x00,0x60,0x60,0x00,0x00,0x00],"?":[0x02,0x01,0x51,0x09,0x06,0x00],
           "@":[0x32,0x49,0x79,0x41,0x3E,0x00],";":[0x00,0x56,0x36,0x00,0x00,0x00],
           "!":[0x00,0x00,0x4F,0x00,0x00,0x00],"(":[0x00,0x1C,0x22,0x41,0x00,0x00],
           ",":[0x00,0x50,0x30,0x00,0x00,0x00],
           "+":[0x1E,0x30,0x78,0x60,0x00,0x00],"-":[0x00,0x00,0x60,0x78,0x30,0x1E],
           "*":[0x30,0x38,0x1C,0x14,0x04,0x00],"/":[0x00,0x04,0x14,0x1C,0x38,0x30],
           "%":[0x3C,0x4A,0x99,0xF9,0xF9,0x99,0x4A,0x3C],
           "=":[0x3C,0x42,0x81,0x81,0x81,0xBC,0x42,0xBD,0x81,0x81,0xBC,0x42,0xBD,
                0x81,0x81,0xBC,0x42,0xBD,0x81,0x81,0x81,0x42,0x3C],
}

lookup = [0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf]
def reverse(n):
  return (lookup[n&0b1111] << 4) | lookup[n>>4]

dis_his=[]
def display_str(strip, str_to_dis, color):
  pos = (64 - len(str_to_dis)*6)/2
  blackout(strip,dis_his)
  del dis_his[:]
  char_list = list(str_to_dis)
  if str_to_dis not in ["<<<<",">>>>"]:
    char_list.reverse()
    for char in char_list:
      if char in ascii_map.keys():
        if char in ['+','%','-','*','%','/','=']:
          seq_list = ascii_map[char]
          if char == '=':pos=20
          for code_seq in range(len(seq_list)):
            if code_seq < len(seq_list): code = seq_list[len(seq_list)-code_seq-1]
            else:  code = seq_list[code_seq]
            if pos%2 == 0:
              code = reverse(code)
            for seq in range(8):
              if ((code >> seq)&0x01) == 1:
                strip.setPixelColor((pos*8+seq), color)
              dis_his.append(pos*8+seq)
            pos += 1
            if (pos+1) == 63: pos=2
        else:  
          seq_list = ascii_map[char]
          for code_seq in range(len(seq_list)):
            if code_seq < 5: code = seq_list[4-code_seq]
            else:  code = seq_list[code_seq]
            if pos%2 == 0:
              code = reverse(code)
            for seq in range(8):
              if ((code >> seq)&0x01) == 1:
                strip.setPixelColor((pos*8+seq), color)
              dis_his.append(pos*8+seq)
            pos += 1
            if (pos+1) == 63: pos=2 
    strip.show()
    time.sleep(0.3)
  elif (str_to_dis == "<<<<" and host_ip == "192.168.1.66") or (str_to_dis == ">>>>" and host_ip == "192.168.1.65"):
    if host_ip == "192.168.1.65": char_list = ["<","<","<","<"]
    for char in char_list:
      if char in ascii_map.keys():
        seq_list = ascii_map[char]
        for code_seq in range(len(seq_list)):
          if code_seq < 5: code = seq_list[4-code_seq]
          else:  code = seq_list[code_seq]
          if pos%2 == 0:
            code = reverse(code)
          for seq in range(8):
            if ((code >> seq)&0x01) == 1:
              strip.setPixelColor((pos*8+seq), color)
            dis_his.append(pos*8+seq)
          pos += 1
          if (pos+1) == 63: pos=2
      strip.show()  
      time.sleep(0.3)  

  elif (str_to_dis == ">>>>" and host_ip == "192.168.1.66") or (str_to_dis == "<<<<" and host_ip == "192.168.1.65"):
    pos = (64 - len(str_to_dis)*6)/2 + len(str_to_dis)*6
    if host_ip == "192.168.1.65": char_list = [">",">",">",">"]
    for char in char_list:
      if char in ascii_map.keys():
        seq_list = ascii_map[char]
        for code_seq in range(len(seq_list)):
          code = seq_list[code_seq]
          if pos%2 == 0:
            code = reverse(code)
          for seq in range(8):
            if ((code >> seq)&0x01) == 1:
              strip.setPixelColor((pos*8+seq), color)
            dis_his.append(pos*8+seq)
          pos -= 1
          if (pos-1) == 0: pos=63
      strip.show()  
      time.sleep(0.3)  

data=multiprocessing.Manager().dict()
data['data']=None
data['bright']=LED_1_BRIGHTNESS
data['audio']=None


emer_data = multiprocessing.Manager().dict()
emer_data['type'] = "normal" # "normal" or "caution"
emer_data['data'] = None

drive_mode = multiprocessing.Manager().dict()
drive_mode['data'] = None

#dis_str_list = ["HELLO","AUTOPILOT","<<<<",">>>>","CAUTION","PARKING","GOODBYE"]
dis_str_list = ["HELLO","+%-","<<<<",">>>>","CAUTION",">STOP<","GOODBYE","MANUAL"]
#dis_str_list_effect = ["=","+%-","<<<<",">>>>","*%/",">STOP<","GOODBYE"]
str_color_list = [Color(255, 255, 255),Color(66, 230, 205),Color(66, 230, 205),Color(66, 230, 205),
                  Color(255, 0, 0),Color(66, 230, 205),Color(255, 255, 255),Color(255, 255, 255)]

def rec_dis_data(data,emer_data):
  while True:
    print("ready to receice data from port 9090")        
    rev_data,client_addr = server_socket.recvfrom(BUFSIZE)
    rev_data_list = rev_data.split(":")
    if rev_data_list[0] == "data":
      data["data"] = rev_data_list[1]
    elif rev_data_list[0] == "bright":
      try:
        data["bright"] = int(rev_data_list[1])
      except Exception as e:
        data["bright"] = strip1.getBrightness()
    elif rev_data_list[0] == "audio":
      data["audio"] = rev_data_list[1]
    elif rev_data_list[0] == "matrix":
      emer_data["type"] = rev_data_list[1]
    elif rev_data_list[0] == "mode":
      drive_mode['data']= rev_data_list[1]
    print("receive data:",rev_data)

def rec_serial_data(emer_data):
  try_count=0
  emer_data_his = None
  while True:
    try:
      print "start receive data from serial data"
      try_count=0
      ser = serial.Serial('/dev/ttyUSB0',115200)  
      while(True):
        print "wating to rev data"
        ser.inWaiting()
        ser_rec_data = ser.readline().strip()
        ser_rec_data_list = ser_rec_data.split(",")
        print ser_rec_data
        if len(ser_rec_data_list)<1 or ser_rec_data_list[0][-7:] != "$STATUS": continue
        if ser_rec_data_list[2][-1] == "1":
          emer_data["type"] = "caution"
          if host_ip == "192.168.1.66":
            send_socket.sendto("matrix:caution",("192.168.1.65",9090))
        else:
          emer_data["type"] = "normal"
          if host_ip == "192.168.1.66" and emer_data_his == "caution":
            send_socket.sendto("matrix:normal",("192.168.1.65",9090))
        emer_data_his = emer_data["type"]
        time.sleep(0.1)
    except Exception as e:
      print e 
      try_count+=1
      print str(try_count)+"th to restart serial port"
      time.sleep(5)

def send_serial_data():
  serial_data = "$RPI,01"
  while True:
    try:
      ser = serial.Serial('/dev/ttyUSB0',115200)
      while True:
        ser.write(serial_data)
        cycle_cnt = int(serial_data[-2:])
        cycle_cnt += 1
        if cycle_cnt ==16 : cycle_cnt = 1
        serial_data = serial_data[:-2]+str(cycle_cnt) if cycle_cnt>9 else serial_data[:-2]+"0"+str(cycle_cnt)
        time.sleep(1)  
    except Exception as e:
      print e 
      print "try to restart serial port"
      time.sleep(10)


def play_rev_data(data):
  global emer_data
  while(True):
    if emer_data["type"] == "caution":
      os.system("sudo mplayer /home/pi/audio/take_over.MP3 >/dev/null")
      time.sleep(0.2)
    if data['audio']:
      os.system("sudo mplayer /home/pi/audio/"+data['audio']+".MP3 >/dev/null")
      data['audio'] = None
    time.sleep(0.8)

def to_dis_status():
  global data
  global emer_data
  global drive_mode
  his_dis=None
  equal_flag=False
  dis_time = 0
  while True:
    #print data
    to_dis_data = data["data"]
    if drive_mode['data'] == 'manual':  to_dis_data = '7'
    if emer_data['type'] == 'caution': to_dis_data = '4'
    if to_dis_data:
      if to_dis_data==his_dis:
        equal_flag = True
        if his_dis not in ["<<<<",">>>>",'caution',"2",'3','4'] and data["bright"] == strip1.getBrightness():
          time.sleep(1)
          continue
      else:
        equal_flag = False
      try:
        his_dis = to_dis_data
        if data["bright"] != strip1.getBrightness():
          strip1.setBrightness(data["bright"])
          strip2.setBrightness(data["bright"])
          equal_flag = False
        if to_dis_data.isdigit() and int(to_dis_data)<len(dis_str_list):
          data_num = int(to_dis_data)
          if not equal_flag:
            if data_num == 2 or data_num == 3: dis_time = int(time.time())
            if data_num in [2,3]:
              p = multiprocessing.Process(target=multiColorWipe,args=(strip2,str_color_list[data_num],dis_str_list[data_num]))
              p.start()
            display_str(strip1,dis_str_list[data_num], str_color_list[data_num])
            if data_num not in [2,3]:
              multiColorWipe(strip2,str_color_list[data_num],dis_str_list[data_num],equal = equal_flag)
          if equal_flag and (data_num == 2 or data_num == 3 or data_num == 4):
            now_time = int(time.time())
            #print now_time,dis_time
            if now_time-dis_time > 5 and (data_num == 2 or data_num == 3):
              display_str(strip1,dis_str_list[data_num], str_color_list[data_num])
              dis_time = now_time
            elif data_num == 4:
              multiColorWipe(strip2,str_color_list[data_num],dis_str_list[data_num],equal = equal_flag)

        elif len(to_dis_data) < 11:
          color = Color(random.randrange(1,255), random.randrange(1,255), random.randrange(1,255))
          if not equal_flag: 
            if to_dis_data=="<<<<" or to_dis_data==">>>>": dis_time = int(time.time())
            display_str(strip1,to_dis_data,color)
            multiColorWipe(strip2,color,to_dis_data,equal = equal_flag)
          if equal_flag and (to_dis_data=="<<<<" or to_dis_data==">>>>"):
            now_time = int(time.time())
            if now_time-dis_time > 5:
              display_str(strip1,to_dis_data,color)
              dis_time = now_time       
      except Exception as e:
        print(e) 
        server_socket.close()
        exit(0)
    if to_dis_data != "CAUTION" and to_dis_data != "4":
      time.sleep(1)

if __name__ == '__main__':
  parser=argparse.ArgumentParser()
  parser.add_argument('-m',type=int,help='mode to set',default=0)
  args=parser.parse_args()

  strip1 = Adafruit_NeoPixel(LED_1_COUNT, LED_1_PIN, LED_1_FREQ_HZ,
                             LED_1_DMA, LED_1_INVERT, LED_1_BRIGHTNESS,
                             LED_1_CHANNEL, LED_1_STRIP)

  strip2 = Adafruit_NeoPixel(LED_2_COUNT, LED_2_PIN, LED_2_FREQ_HZ,
                             LED_2_DMA, LED_2_INVERT, LED_2_BRIGHTNESS,
                             LED_2_CHANNEL, LED_2_STRIP)
  strip3 = Adafruit_NeoPixel(LED_1_COUNT, LED_3_PIN, LED_1_FREQ_HZ,
                             LED_1_DMA, LED_1_INVERT, LED_1_BRIGHTNESS,
                             LED_1_CHANNEL, LED_1_STRIP)

  strip4 = Adafruit_NeoPixel(LED_2_COUNT, LED_4_PIN, LED_2_FREQ_HZ,
                             LED_2_DMA, LED_2_INVERT, LED_2_BRIGHTNESS,
                             LED_2_CHANNEL, LED_2_STRIP)
  strip1.begin()
  strip2.begin()
  strip3.begin()
  strip4.begin()
  signal.signal(signal.SIGINT, _signal_handler) # 2
  signal.signal(signal.SIGTERM, _signal_handler) # 15
  print('Press Ctrl-C to quit.')

  if args.m==0:
    p1 = multiprocessing.Process(target=rec_dis_data,args=(data,emer_data, drive_mode))
    p1.start()
    p2 = multiprocessing.Process(target=to_dis_status)
    p2.start()
    if host_ip == "192.168.1.66":
      p3 = multiprocessing.Process(target=rec_serial_data,args=(emer_data,))
      p3.start()
      p4 = multiprocessing.Process(target=send_serial_data)
      p4.start()
      p5 = multiprocessing.Process(target=play_rev_data,args=(data,))
      p5.start()
  else:
    while True:
      for i in range(len(dis_str_list)):
        if dis_str_list[i] in ["<<<<",">>>>"]:
          p4 = multiprocessing.Process(target=multiColorWipe,args=(strip2,str_color_list[i],dis_str_list[i]))
          p4.start()
        display_str(strip1,dis_str_list[i], str_color_list[i])
        if dis_str_list[i] not in ["<<<<",">>>>"]: multiColorWipe(strip2,str_color_list[i],dis_str_list[i],bias=0)
        time.sleep(2)
  p1.join()
  p2.join()
  p3.join()
  p4.join()
  p5.join()
