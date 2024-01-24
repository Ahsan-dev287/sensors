'''
This is a basic driver for the rotary encoder AS5048B high resolution position sensor.
The driver is for I2C interface.
Tested on Raspberry pi 4 (OS Bullseye 32-bit)

'''

import time
from smbus import SMBus


# I2C address of the sensor
AS5048B_ADD = 0x40

# All I2C registers (But not all are used in this code)
AS5048B_PROG_REG        = 0x03
AS5048B_ADDR_REG        = 0x15 
AS5048B_ZEROMSB_REG     = 0x16 #bits 0..7 
AS5048B_ZEROLSB_REG     = 0x17 #bits 0..5 
AS5048B_GAIN_REG        = 0xFA 
AS5048B_DIAG_REG        = 0xFB
AS5048B_MAGNMSB_REG     = 0XFC #bits 0..7 
AS5048B_MAGNLSB_REG     = 0xFD #bits 0..5 
AS5048B_ANGLMSB_REG     = 0xFE #bits 0..7 
A55048B_ANGLLSB_REG     = 0xFF #bits 0..5 
AS5048B_RESOLUTION      = 16384.0 # 14 bits 2 ^ 14

bus = SMBus(1)

def read_angle_regs_14bit(): 
    angle_msb = bus.read_i2c_block_data(AS5048B_ADD, AS5048B_ANGLMSB_REG, 1) 
    time.sleep(0.2) 
    angle_lsb= bus.read_i2c_block_data(AS5048B_ADD, A55048B_ANGLLSB_REG, 1) 
    concat_angle = ((angle_msb[0]) << 6)                    # this moves the bits 6 places to the left
    concat_angle = (concat_angle | (angle_lsb[0]&0x3F))     # 0x3F is equal 111111
    
    return angle_msb[0], angle_lsb[0], concat_angle 

def zero_reg_write(msb , lsb): 
    bus.write_byte_data(AS5048B_ADD, AS5048B_ZEROMSB_REG, msb) 
    time.sleep (0.2) 
    bus.write_byte_data(AS5048B_ADD, AS5048B_ZEROLSB_REG, lsb) 
     
def convert_angle(angle): 
    angle = (angle/AS5048B_RESOLUTION) * 360.0 
    return round(angle, 2)


if __name__=="__main__":

    zero_reg_write(0x00, 0x00)                                     # write 0 to zero position registers to reset
    angle_msb , angle_lsb, _ = read_angle_regs_14bit()             # read angle registers
    zero_reg_write(angle_msb, angle_lsb)                           # write the angle register to make this position your zero position.

    while True:
        
        _ , _ , concat_angle = read_angle_regs_14bit()
        final_angle = convert_angle(concat_angle)
        print(f"Rotary Encoder Heading : {final_angle}")
