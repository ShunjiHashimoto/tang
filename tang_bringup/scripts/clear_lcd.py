import I2C_LCD_driver
from time import *

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_display_string("This is how you", 1)
sleep(1)

mylcd.lcd_clear()

mylcd.lcd_display_string("clear the screen", 1)
sleep(1)

mylcd.lcd_clear()
