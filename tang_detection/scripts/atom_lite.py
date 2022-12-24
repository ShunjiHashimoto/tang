import serial
import time

usb_device = serial.Serial('/dev/ttyUSB0', '9600', timeout=1.0)

def main():
    command = "Manual"
    while(True):
        try:
            print("Write signal")
            usb_device.write(command.encode())
            time.sleep(1.0)
        except:
            print("close usb_device", flush=True)
            usb_device.close()
            return

if __name__ == "__main__":
    main()
    