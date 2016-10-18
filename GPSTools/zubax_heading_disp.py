import os
import serial

def main():
	try:
		ser = serial.Serial("/dev/ttyACM0", '57600')
	except:
		print("Error opening serial port!!")
		exit()

	print("Reading data...")

	while True:

		st = ser.read(1)
		# print(st)
		if st == b'$':
			# print("sfsd")
			msg = ser.readline().decode('utf-8')
			# print(msg)

			contents = msg.split(',')

			if contents[0] == 'HCHDG':

				heading = float(contents[1])
				print(heading)

if __name__ == '__main__':
	main()