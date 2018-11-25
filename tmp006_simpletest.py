import board, busio
import adafruit_tmp006

i2c = busio.I2C(board.SCL, board.SDA)

tmp = adafruit_tmp006.TMP006(i2c)