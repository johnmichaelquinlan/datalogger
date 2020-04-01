import RPi.GPIO as GPIO
import subprocess
import psutil
import sys
import serial

from time import sleep
from datetime import datetime

captureButton = 17
clueStartCapture = 22
clueStopCapture = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup( captureButton, GPIO.IN, GPIO.PUD_DOWN )
GPIO.setup( clueStartCapture, GPIO.OUT )
GPIO.output( clueStartCapture, GPIO.HIGH )
GPIO.setup( clueStopCapture, GPIO.OUT )
GPIO.output( clueStopCapture, GPIO.HIGH )


ser = serial.Serial( port='/dev/ttyAMA0', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize= serial.EIGHTBITS, timeout=1 )
captureProcessHandle = None
timeStamp = None
outputfile = None
capturing = False


def StartClueCapture():
	global timeStamp
	global capturing
	global outputfile
 	datalogfilename="/home/pi/datalog/"+timeStamp+"_datalog.json"
        outputfile = open(datalogfilename,"a")
	GPIO.output(clueStartCapture, GPIO.LOW)
	sleep(0.1)
	GPIO.output(clueStartCapture, GPIO.HIGH)
	capturing = True

def SyncFromClue():
        global outputfile
        received = ser.read()
        outputfile.write(received)

def StopClueCapture():
	global capturing
	global outputfile
	capturing = False
	GPIO.output(clueStopCapture, GPIO.LOW)
	sleep(0.1)
	GPIO.output(clueStopCapture, GPIO.HIGH)
	outputfile.flush()
	outputfile.close()

def StopProcessTree( processHandle ):
	if processHandle.poll() == None:
                for child in psutil.Process(processHandle.pid).children(recursive=True):
                        child.terminate()
                processHandle.terminate()
        processHandle = None

def BeginCapture():
	global captureProcessHandle
	global timeStamp
	print ("Starting video capture...")
	captureProcessHandle = subprocess.Popen(["/home/pi/scripts/videocapture", timeStamp])
	print ("Starting clue capture...")
	StartClueCapture()

def EndCapture():
	global captureProcessHandle
	print ("Stopping video capture...")
	StopProcessTree(captureProcessHandle)
	print ("Stopping clue capture...")
	StopClueCapture()

def main():
	global capturing
	global timeStamp
	print ("Ready")
	ser.write("ready\n")
	try:
		previousValue = 0
	        while True :
        	        currentValue = GPIO.input( captureButton )
                	buttonActivated = currentValue == 1 and previousValue == 0
	                buttonDeactivated = currentValue == 0 and previousValue == 1
        	        previousValue = currentValue
                	if buttonDeactivated:
                        	EndCapture()
				print ("Ready")
	                elif buttonActivated:
				dateTimeNow = datetime.now()
        			timeStamp = dateTimeNow.strftime("%Y-%m-%d_%H-%M-%S")
        	                BeginCapture()

			if capturing:
				SyncFromClue()

	finally:
	        GPIO.cleanup()


main()
