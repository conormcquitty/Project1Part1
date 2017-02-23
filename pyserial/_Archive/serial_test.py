'''
python server side program for testing serial communications
with an arduino

Before running the script, upload SimpleClient to your Arduino.

Then run this script with:
   python3 serial_test.py [-s <serial-port>]

   -s allows you to specify the serial port to be used.
     if no port is specified, the standard input/output will be used.

You can see what serial ports are plausible with
   arduino-port-select
On the Ubuntu VM, the serial-port will most likely be /dev/ttyACM0,
while it will be /dev/tty.usbmodem???? on the Mac

'''

# for testing with stdin/stdout, platform
import sys
import argparse

# this imports serial, and provides a useful wrapper around it
import textserial

# If you are on OSX you need to get the serial module:
#
#   sudo pip install pyserial
#
# For installing pip, see:
# http://stackoverflow.com/questions/17271319/installing-pip-on-mac-os-x
# and
# https://pip.pypa.io/en/latest/installing.html#install-or-upgrade-pip
#
# If you feel adventurous, you may try the following, if the above did
# not work: sudo easy_install pyserial


def parse_args():
    '''
    Parses arguments for this program.

    Returns:
    An object with the following attributes:
     serialport (str): what is after -s or --serial on the command line
    '''

    DEF_PORT = ('/dev/tty.usbmodem1411'
                if sys.platform == 'darwin' else
                '/dev/ttyACM0')

    parser = argparse.ArgumentParser(
        description='Serial port communication testing program.',
        epilog='If the port is 0, stdin/stdout are used.\n'
    )
    parser.add_argument('-s', '--serial',
                        help='path to serial port '
                             '(default value: "%s")' % DEF_PORT,
                        dest='serialport',
                        default=DEF_PORT)

    return parser.parse_args()


def data_transfer(serial_in, serial_out):
    '''
    Example code that waits for lines from the client and responds.
    To be used with SimpleClient.cpp

    Args:
        serial_in: stream that this function is reading from
        serial_out: stream that this function writes to
    '''

    print("Data transfer started.")

    # Read garbage until "PHASE01" is received:
    line = ""
    while line != "PHASE01":
        # it's possible for bad characters to sneak in and confuse the
        # byte codec (0-127 char range), especially if you reset the
        # arduino, so ignore those

        try:
            line = serial_in.readline()
            line = line.rstrip('\r\n')  # remove trailing newline
            print("Read a line: <%s>" % line)
        except Exception as e:
            print(e)
            print("Skipping this bad data.")

    print("Entering next phase")
    # Read the introductory lines until "PHASE02" is received
    for line in serial_in:
        line = line.rstrip('\r\n')
        print("Read a line: <%s>" % line)
        if line == "PHASE02...":
            # acknowledge, to get the next piece of data
            print("Acknowledged", file=serial_out)
            print("Acknowledged")
            break

    # Read further lines and respond to them by sending an index
    print("Entering next phase")
    idx = 0
    for line in serial_in:
        line = line.rstrip('\r\n')
        print("Line received: <%s>" % line)
        # send back data
        if idx > 10:
            break
        print(idx, file=serial_out)
        idx += 1


def main():
    '''Main code to illustrate the usage of TextSerial.
    '''
    args = parse_args()

    if args.serialport != "0":
        print("Opening serial port: %s" % args.serialport)
        baudrate = 9600  # [bit/seconds] 115200 also works
        with textserial.TextSerial(args.serialport, baudrate, newline=None) \
                as ser:
            data_transfer(ser, ser)

        # with serial.Serial(args.serialport, baudrate) as serial_out:
        #     # with serial.Serial(args.serialport, baudrate) as serial_in:
        #         data_transfer(serial_out,serial_out)
    else:
        print("No serial port. Using stdin/stdout.")
        data_transfer(sys.stdin, sys.stdout)

    print("Demo finished.")


if __name__ == '__main__':
    main()