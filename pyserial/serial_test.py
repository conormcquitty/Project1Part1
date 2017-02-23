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

    cur_phase = "PHASE00"
    next_phase = "PHASE01"

    print("Server started.  You might need to reset your arduino.")
    print("Server is in phase {}".format(cur_phase))
    print("Waiting for PHASE01 message.")

    # Read garbage until "PHASE01" is received:
    line = ""
    while line != next_phase:
        # it's possible for bad characters to sneak in and confuse the
        # byte codec (0-127 char range), especially if you reset the
        # arduino, so ignore those

        try:
            line = serial_in.readline()
            line = line.rstrip('\r\n')  # remove trailing newline
            print("Received <{}>".format(line))
        except Exception as e:
            print(e)
            print("Skipping bad data.")

    cur_phase = next_phase
    next_phase = "PHASE02"
    print("Server is in phase {}".format(cur_phase))

    # Read the introductory lines until "PHASE02" is received
    for line in serial_in:
        line = line.rstrip('\r\n')
        print("Received <{}>".format(line))
        if line == next_phase:
            # acknowledge, to get the next piece of data
            msg = "Ack"
            print("Sending <{}>".format(msg))
            print(msg, file=serial_out)
            break

    cur_phase = next_phase
    next_phase = "PHASE00"
    print("Server is in phase {}".format(cur_phase))

    # Read further lines and respond to them by sending an index
    print("Waiting for client reply to previous message.")

    idx = 0
    for line in serial_in:
        line = line.rstrip('\r\n')
        print("Received <{}>".format(line))

        # send back data
        if idx > 10:
            print("Exchange finished")
            break

        msg = str(idx)
        print("Sending <{}>".format(msg))
        print(msg, file=serial_out)
        idx += 1

    cur_phase = next_phase
    print("Server is in phase {}".format(cur_phase))


def main():
    '''Main code to illustrate the usage of TextSerial.
    '''
    args = parse_args()

    if args.serialport != "0":
        print("Opening serial port: %s" % args.serialport)
        baudrate = 9600  # [bit/seconds] 115200 also works

        # The with statment ensures that if things go bad, then ser
        # will still be closed properly.

        with textserial.TextSerial(
            args.serialport, baudrate, newline=None) as ser:
                data_transfer(ser, ser)

        # with serial.Serial(args.serialport, baudrate) as serial_out:
        #     with serial.Serial(args.serialport, baudrate) as serial_in:
        #         data_transfer(serial_out,serial_out)

    else:
        print("No serial port. Using stdin and stdout.")
        data_transfer(sys.stdin, sys.stdout)

    print("Demo finished.")


if __name__ == '__main__':
    main()
