# -*- coding: utf-8 -*-
#!/usr/bin/env python3
 
import I2C_LCD_driver
from time import *

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("Konnichiwa!!!!!", 1)

