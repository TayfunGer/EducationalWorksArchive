#import enum
from icmplib import ping, traceroute
import icmplib
# from sys import argv
from argparse import ArgumentParser

# Check argv to make it optional with default values
parser = ArgumentParser()
parser.add_argument("-a", "--address", dest="address", default="127.0.0.1", type=str)
parser.add_argument("-m", "--mode", dest="mode", default="ping", type=str)
parser.add_argument("-c", "--count", dest="count", default="5", type=int)
parser.add_argument("-i", "--interval", dest="interval", default=1, type=float)
args = parser.parse_args()

# send ping count times in a intervall or a traceroute to a IP-address
if args.mode == "traceroute":
    client = traceroute(args.address)
else:
    client = ping(args.address, count=args.count, interval=args.interval)
print(client)