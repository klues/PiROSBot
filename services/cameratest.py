from picamera import PiCamera
from time import sleep

camera = PiCamera()

try:
    camera.start_preview()
    sleep(1)
    camera.stop_preview()
    pass
finally:
    camera.close()
