#!/usr/bin/env python3
#####################################################################################
# Filename    : LogicAnalyzer_UART.py
# Description : Logic_Analyzer_With_UART
# Author      : Bob Fryer / Digital Shack
# modification: 22 Feb 2020
#####################################################################################
#####################################################################################
# Import the required Python Libraries
#####################################################################################
import  serial, time 
import  RPi.GPIO as GPIO
#####################################################################################
# Initialise the Serial Port (ttys0) and flush any serial data from the buffers
#####################################################################################
ser  =  serial.Serial(port  =  "/dev/serial0", baudrate = 9600, timeout = 1000 , parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS  )
#ser2  =  serial.Serial(port  =  "/dev/serial2", baudrate = 9600, timeout = 1000 , parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)
if  (ser.isOpen()  ==  False):
    ser.open()                                       # check and open Serial0
ser.flushInput()                                       # clear the UART Input buffer
ser.flushOutput()                                      # clear the UART output buffer
#if  (ser2.isOpen() == False):
#    ser2.open()
#ser2.flushInput()
#ser2.flushOutput()
#, baudrate = 9600, timeout = 2 , parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS 
#####################################################################################
# Define our one and only variable
#####################################################################################
#echostring  =  "Talking to ourselves!"
#echostring = "test"
#data = [0,0,0,0,0,1];
#motorx ='y0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000011111111111111111111111111111111111111111111111111111111111110000000000000001000010010010100100100100101001010100101010101011011111111111111111111111111111111111111111010110101010010101001010010010010100100nn0nn0n0n0n0n0n00n0n00n0000n00001000100100101010101010100n0n0n0n0n0nn0nnnnnnnnnnnnnnnnnnnnnnnnnnn000001nnn001000100101001010101011010111111110000000n00n00n00n0n0n0n0n0nn0nn0nnn000000nnnnnnnnnnnnnnnnnnnnnnnnnnnn0nn0n0nn0n0n0n0n00n00n0n00n0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn'
#motory = 'y11111n'
data = 'x1111111111111110000000000000000010000000000n00000000000000000n0000000000000000000001111111111000000000000nnnnnnnnn00000000000000000011111111111111111111111111111111111111111111111111111111111111000n000000000000000000000000000nnnnnnnnnn1000000n000000000nnnnnnnnnnnnnnnnnnn00000000nnnnnnnnn00000000nnnnnnnnnnnnnn000000n000000000nnnnnnnn000100000000000000000n00011111111000000001000000111111111111110000001111111110000000111111111111111111100000000100000011111111000000000000nnnn0nnnnnnnnn0nnnn0nnnnnnnnnnn0nnnnn0nnnnnnnnnnnnnnnnnnnn0nnn0n0111111111111111111111111111101111111111111111101110nnn0nnnnnnnnnnnnnnnnn0nnnnnnnnnnnnnnnnnnnnnnnnnnnn1111111111111111111111111111011111111111111111111111111111100000000nn0nn0nn0nnnnnnnnnn0nnnnnn000000000000000001011110111111110111111111111000001000nn0nnnnnnnnnn01111111110111000000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn00000010n001111111111011111111110nnnnnnnnnnnnnnnnn00000000111111111111111111111111110111111111111111111110nnnnnnnnnnnnnnnnnnn0nnnnnnnnnnnnnnnnnnnnnnnnnnn0000001111111111111111110100n010nnnnnnnnnnnnnnnnnnnnnn0010n00000111111111111111111111111111111111111111111111111111111111111000100000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn11111111111111111111110110110nnnnnnnnnnnnnnnnnnnnnnnn0nn01011111111111111111111111111111111111111111111000010000000nnnnnnnnnnn0111111111101000000n000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn111111111111111111111111100nnnnnnnnnnnnnnnnnnnnnnnnnnn1111111111111111111111111111111111111111111110000n0010n100000n100n0000nnnnnnnnnnnnnnnnnnnnnnnnn000000000000000011111111111111111111111111111111111111111111111100000000000000000001111111111111111111110000000001000000000000000n000000000nnnnnnnnnn0nnnnnnnnnnn00000000000000000001011111000000n00000111111111111000000010001011111111111111111101001000000000n0000n000n0nnnnn0100000000000100n00n0nnnnnn01111110010010n00000000000n11111100100001000000000n00nnnnnnnnnnnnnnnnnnnn000n0000000nnnnnnnnnnnn00000100000010nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn000000n0000000000000100000000000000000000000000100nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn0000000000111110000001000000000000111111111111111111111111111111111111111111111111000n010000000nnnnnnnnnnnnnnn01111111111111101000000n01000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn00000000000n00000nnnnnnnnnnnnnnn000000000000000000001000000000000000010000000000n1111111111111111110000000000000000000001111111111111111111111111110nnnnnnnnnnnnnnnnnnnnnnnnnnn0000000000000000000nnnnnnnnnnnnnnnnnny0000000000000001111111111111111111111111111111111111111111111011111111111111111111100100000011111111111110100000001111111110nnnnnnnnn000000nn1000000000000000000010000000000n0000000000000000n000011111111111111111111111111111110000000000nnnnnnnnnnnnnnnnn00000000000000000001111111100000n000nnnnnnnn0000000000n00011111111111111110000000nnnnnnnnnnnnn01111111111111000000nnnnnnnnnnnnnnn0n001000000000111111110010000nnnnnnn0n000000000000000001111111111111111100000011111111111110010101001010010100101010111010101010101010101010101010100101010111001010001001010101010010101n1010010010101010101000n00nn0n0n0n00n00n0n10nn0n00n0n0n0n0n00n000n0n00nnnnnn00n0n0n0n0n0n0n0n0nn0n0n0n0n0nnn0n0nn00nn00n0n00nn0n001111111110010010010101010100101010111111111111111111010100101010110101010100101011111111100n0nn00n0nn00011010011001111111100000000000001000000000100000000000001000000100000001000100001111111111111010101001001001010110010001001001001011111111001000100010010000100100101000100010010010010000000n00n00n00n000n000n0n00n00n0000n00n000n000n0nnnnnnnnn00n00n00n000n0n010n010n000n0n0n00n0n00n0n0n0nnnnnnnnnnnnn000n000n0000000n000000n0000000000000n000000000n000000000001111011111100000100000001000000010000001000000010000001100100101001010101010010010110001000100010001000010001000000100001000100010001000100100010000100010001011111111111100n00n00n00000100100100nnnnnnnnnnnnn000n000n0000n000n00n000n000n000n000n0000nn00n0000n000n000n000n00n010nnn0n00n0n0n0n0n00n0n00n00nn000000n0000000n000000n0000000n0000000n00000nnnnnnnnn0nnnnnnnnnnnnnnnnnn0n00n0n0nn0nn0n0nn00nnnnnnnnnnnnnnnnnnnn0n0nn00n0n0n0nn0n0n0n0000000n00000000000n000000nnnnnnnnnnnnnnnnnnnn0000000n000000000001111111111111111110nnnnnnnnnnnnnnnnnn0000000n0010100000001111111111111111111111000nnnnnnnnnnnnnnn0000000000111111111111110100000000n00n0000nnnnnnnnnnnnnnnnnnnnnnn0n0001011111111111111110110101000000000n00nn0nnnnnnnnnnnnnnnnnn00111111111111111111111111000100100000000nnnnnnnnnnnnnnn0000000000111111111111110n1100nn0000001000000000001000000nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn0001000000000000000010000000000n0000000000000000000nnnnnnnnnnnn0000nnnnnnnnnnnnnnnnnnn00100000000000000000000000000000010000000001000nnnnnnnnnnnnnn000n00000000000000000000000100011111111111111000n000000000n000000000000000000000000000000n011111111111111111110000n000000n0nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn0000n000000000010111111111111111111111000000000000000000000000100000n00000000000000000000000nnnnnnnnnnnnnnnnnnnnnn000000000010000n'
#####################################################################################
# Define our main task
#####################################################################################
def echotest(data):
    while (True):                                      # loop
        #ser.flushInput()
        #ser.flushOutput()
        #ser.write(str(size).encode());
        #print ('size of data: ', size)
        ser.flushInput()                    # Clear UART Input buffer
        ser.flushOutput()                              # Clear UART Output buffer
        ser.write(str(data).encode())                 # write our String in bytes
        print ('array sent: ', data)           # Print what we sent
#        time.sleep(10)                                # give it a sec to be recvd
#        ser.close()
#        ser2.flushInput()
#        ser2.flushOutput()
#        ser2.write(str(motory).encode())
#        print ('motory sent: ', motory)
        time.sleep(5)
        break
        #echostringrec = ser.readline().decode("utf-8") # read the rec buffer as UTF8
        #print ('What we received: ', echostringrec)    # Print what we received
#####################################################################################
# Define our DESTROY Function
#####################################################################################
def destroy():
    ser.close ()                                          # Closes the serial port
    print ("test complete")
#####################################################################################
# Finally the code for the MAIN program
#####################################################################################
if __name__ == '__main__':                                 # Program entry point
    try:
        echotest(data)                                         # call echotest function
        #echotest(motory)
    except KeyboardInterrupt:                              # Watches for Ctrl-C
        destroy()                                          # call destroy function
    finally:
        destroy()                                          # call destroy function
