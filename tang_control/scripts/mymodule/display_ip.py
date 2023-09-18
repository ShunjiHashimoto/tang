#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import time
import RPi.GPIO as GPIO
import lcd_display
import socket
import fcntl
import struct
import subprocess

mylcd = lcd_display.lcd()

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915, 
        struct.pack('256s', bytes(ifname[:15], 'utf-8'))
    )[20:24])

def check_ping():
    raspi_ip = get_ip_address("eth0")
    # jetson_ip = "192.168.0.104"
    jetson_ip = "192.168.2.101"
    hosts = [jetson_ip, raspi_ip]
    for host in hosts:
        res = subprocess.run(["ping",host,"-c","5", "-W", "1000"],stdout=subprocess.PIPE)
        print(res.stdout.decode("cp932"))
        print("result:", res.returncode)
        print("host: ", host)
        if(res.returncode == 0):
            return 0
    return 1

mylcd.lcd_display_string("ubuntu@raspi", 1) 
mylcd.lcd_display_string(get_ip_address('wlan0'), 2)
# check ping
result = 1
start = time.time()
diff  = 0
while(result != 0 or diff < 10):
    result = check_ping()
    mylcd.lcd_display_string("connected", 2)
    end  = time.time()
    diff = end - start
