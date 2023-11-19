import gb_icmplib as icmp
import socket
from argparse import ArgumentParser
from time import time
import select
from random import randint

default_message = """000102030405060708090A0B0C0D0E0F
101112131415161718191A1B1C1D1E1F
202122232425262728292A2B2C2D2E2F
303132333435363738""".replace('\n', '')

# Check argv to make it optional with default values
parser = ArgumentParser()
parser.add_argument("-a", "--addresses", dest="addresses", default="127.0.0.1", type=str)
parser.add_argument("-i", "--interval", dest="interval", default=1, type=float)
parser.add_argument("-d", "--data", dest="data", default=default_message, type=str)
args = parser.parse_args()

raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)

print("\nStart multiping")

for host in args.addresses.split(","):
    id = randint(0, 65535)
    raw_data = icmp.EchoMessage(id, 1, hex(int(args.data, base=16))[2:]).generate_raw_data()
    start = time()
    raw_sock.sendto(raw_data, (host, 0))
    while time() < start + args.interval:
        ready = select.select([raw_sock], [], [], start - time() + args.interval)
        if ready[0]:
            data, addr = raw_sock.recvfrom(1024)
        else: 
            print("[-] Host " + host + " is down or is filtering")
            break

        icmp_header = icmp.ICMPHeader()
        icmp_header.load_raw_data(data)

        if icmp_header.type == icmp.MessageTypes.ECHO_REPLY and \
        icmp_header.id == id and \
        icmp_header.sequence_number == 1:
            print("[+] Host " + host + " is up")
            break
print("")