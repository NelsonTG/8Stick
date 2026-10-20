import cv2
import numpy
import time
import board, busio 
import os
import adafruit_mpu6050 as ada

 

i2c = busio.I2C(board.SCL,board.SDA)
mpu1 = ada.MPU6050(i2c, address = 0x68)
mpu2 = ada.MPU6050(i2c, address = 0x69)
cam = cv2.VideoCapture("/dev/video0",cv2.CAP_V4L2)

#Courtesyof mcp3008 library 
import spidev

# Modes Single
CH0 = 8     # single-ended CH0
CH1 = 9     # single-ended CH1
CH2 = 10    # single-ended CH2
CH3 = 11    # single-ended CH3
CH4 = 12    # single-ended CH4
CH5 = 13    # single-ended CH5
CH6 = 14    # single-ended CH6
CH7 = 15    # single-ended CH7
# Modes Diff
DF0 = 0     # differential CH0 = IN+ CH1 = IN-
DF1 = 1     # differential CH0 = IN- CH1 = IN+
DF2 = 2     # differential CH2 = IN+ CH3 = IN-
DF3 = 3     # differential CH2 = IN- CH3 = IN+
DF4 = 4     # differential CH4 = IN+ CH5 = IN-
DF5 = 5     # differential CH4 = IN- CH5 = IN+
DF6 = 6     # differential CH6 = IN+ CH7 = IN-
DF7 = 7     # differential CH6 = IN- CH7 = IN+

RESOLUTION = 1 << 10 # 10 bits resolution

class MCP3008(spidev.SpiDev):
    '''
    Object that listens the MCP3008 in the SPI port of the RPi.
    Connects the object to the specified SPI device.
    The initialization arguments are MCP3008(bus=0, device=0) where:
    MCP3008(X, Y) will open /dev/spidev-X.Y, same as spidev.SpiDev.open(X, Y).
    '''
    def __init__(self, bus=0, device=0, max_speed_hz=976000):
        self.bus = bus
        self.device = device
        self.open(self.bus, self.device)
        self.modes = False
        self.max_speed_hz = max_speed_hz

    def __del__(self):
        self.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()

    def __repr__(self):
        return 'MCP3008 object at bus {0}, device {1}'.format(self.bus, self.device)

    def __call__(self, norm=False):
        return self.read(self.modes, norm)

    @classmethod
    def fixed(cls, modes, bus=0, device=0):
        '''
        Initializes the class with fixed modes, which turns the instance callable.
        The modes argument is a list with the modes of operation to be read (e.g.
        [mcp3008.CH0,mcp3008.Df0]).
        When calling the instance the object will execute a reading of and return the
        values (e.g. print instance()).
        When calling the instance, you can pass the optional argument norm to
        normalize
        the data (e.g. print instance(5.2)).
        '''
        instance = cls(bus, device)
        instance.modes = modes
        return instance

    def _read_single(self, mode):
        '''
        Returns the value of a single mode reading
        '''
        if not 0 <= mode <= 15:
            raise IndexError('Outside the channels scope, please use: 0, 1 ..., 7')
        request = [0x1, mode << 4, 0x0] # [start bit, configuration, listen space]
        _, byte1, byte2 = self.xfer2(request)
        value = (byte1%4 << 8) + byte2
        return value

    def read(self, modes, norm=False):
        '''
        Returns the raw value (0 ... 1024) of the reading.
        The modes argument is a list with the modes of operation to be read (e.g.
        [mcp3008.CH0,mcp3008.Df0]).
        norm is a normalization factor, usually Vref.
        '''
        reading = []
        for mode in modes:
            reading.append(self._read_single(mode))
        if norm:
            return [float(norm)*value/RESOLUTION for value in reading]
        else:
            return reading

    def read_all(self, norm=False):
        '''
        Returns a list with the readings of all the modes
        Data Order:
        [DF0, DF1, DF2, DF3, DF4, DF5, DF6, DF7,
         CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7]
        norm is a normalization factor, usually Vref.
        '''
        return self.read(range(16), norm)
mcp = MCP3008()
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
hit_line_y = 300  
line1 = 160
line2 = 320
line3 = 480
lastswing = 0
lastHit = 0 
hitCooldown = 0.3 

while True:
    ret, frame = cam.read()
    if not ret:
        break
    gx1, gy1, gz1 = mpu1.gyro
    gx2,gy2,gz2 =mpu2.gyro
    

    force= mcp.read([CH0])[0]
    H, W = frame.shape[:2]

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = numpy.array([90, 80, 120])
    upper = numpy.array([130, 255, 255])
    mask  = cv2.inRange(hsv, lower, upper)
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cv2.line(frame, (line1, 0), (line1, H), (255, 255, 255), 2)
    cv2.line(frame, (line2, 0), (line2, H), (255, 255, 255), 2)
    cv2.line(frame, (line3, 0), (line3, H), (255, 255, 255), 2)
    cv2.line(frame, (0, hit_line_y), (W, hit_line_y), (255, 255, 255), 2)

    best = None   
    best_cy = -1

    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        cx = x + w // 2
        cy = y + h // 2
        
        if cy > hit_line_y and cy > best_cy:
            best_cy = cy
            best = (x, y, w, h, cx, cy)
    if gy1 < -.75 or gy2 < -.75 :
        lastswing = time.time()
        if best is not None:
            x, y, w, h, cx, cy = best
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)
            now = time.time()
            if now - lastswing < .3 and now - lastHit > hitCooldown:
                lastHit = now
            if cx < line1:
                os.system("aplay /home/eg1004/Downloads/hightom.wav &")
                time.sleep(.5)
            elif cx < line2:
                os.system("aplay /home/eg1004/Downloads/hihat.wav &")
                time.sleep(.5)
            elif cx < line3:
                os.system("aplay /home/eg1004/Downloads/snaredrum.wav &")
                time.sleep(.5)
            else:
                os.system("aplay /home/eg1004/Downloads/floortom.wav &")
                time.sleep(.5)
    
    if force>250:
        os.system("aplay /home/eg1004/Downloads/bassdrum.wav &")
        time.sleep(.3)
    cv2.imshow("Camera", frame)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
