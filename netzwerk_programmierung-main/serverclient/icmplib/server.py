import icmplib
from time import localtime, strftime
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-a", "--address", dest="address", default=None, type=str)
args = parser.parse_args()

s = icmplib.ICMPv4Socket(address=args.address)

while True:
    try:
        data = s.receive(timeout=10000)
    except Exception:
        continue
    print("New packet at " + strftime("%H:%M:%S\n", localtime()))
    print("Source        : " + str(data._source))
    print("Version       : " + str(data._family))
    print("Bytes received: " + str(data._bytes_received))
    print("Type          : " + str(data._type))
    print("Code          : " + str(data._code))
    #print("ID            : " + str(data._id))
    #print("Sequence      : " + str(data._sequence))
    print("Time          : " + str(data._time) + "\n")
    print("***********************************")
