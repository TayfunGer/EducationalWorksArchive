import gb_icmplib as icmp
import socket
import select
from argparse import ArgumentParser
from time import time

# Check argv to make it optional with default values
parser = ArgumentParser()
parser.add_argument("-a", "--address", dest="address", default="127.0.0.1", type=str)
parser.add_argument("-p", "--port", dest="port", default="33434", type=str)
parser.add_argument("-m", "--max_hops", dest="max_hops", default=30, type=int)
parser.add_argument("-i", "--interval", dest="interval", default=1, type=float)
args = parser.parse_args()

icmp_header = icmp.ICMPHeader()
ttl = 1

try:
    target = socket.gethostbyaddr(args.address)[0]
except Exception:
    target = args.address

print("\n<< Traceroute to " + target + " (" + str(args.max_hops) + " hops max) >>\n")
while True:
    # sockets
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    udp_sock.setsockopt(socket.SOL_IP, socket.IP_TTL, ttl)
    raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    raw_sock.setblocking(0)

    received = False

    # sending udp packet
    start = time()
    udp_sock.sendto(str.encode(""), (args.address, int(args.port)))

    # waiting for reply
    ready = select.select([raw_sock], [], [], args.interval)  # also possible with setsockopt (and try except)
    if ready[0]:
        data, addr = raw_sock.recvfrom(1024)
        end = time()
        received = True

    udp_sock.close()
    raw_sock.close()

    if received:
        # analyse reply
        icmp_header = icmp.ICMPHeader()
        icmp_header.load_raw_data(data)

        # time to live exceeded in transit
        if icmp_header.type == icmp.MessageTypes.TIME_EXCEEDED and   \
        icmp_header.code == 0:
            try:
                hostname = socket.gethostbyaddr(addr[0])[0]
            except Exception:
                hostname = addr[0]

            print("[-]" + "{:2d}".format(ttl) + ". " + "{:>15s}".format(addr[0]) + " - " + hostname + " (" + "{:.3f}".format((end-start)*1000) + "ms)")

        # target reached port not reachable 
        elif icmp_header.type == icmp.MessageTypes.DESTINATION_UNREACHABLE:
            try:
                hostname = socket.gethostbyaddr(addr[0])[0]
            except Exception:
                hostname = addr[0]

            print("[+]" + "{:2d}".format(ttl) + ". " + "{:>15s}".format(addr[0]) + " - " + hostname + " (" + "{:.3f}".format((end-start)*1000) + "ms)")
            break
        
        # target reached port reachable
        else:
            try:
                hostname = socket.gethostbyaddr(addr[0])[0]
            except Exception:
                hostname = addr[0]

            print("[++]" + "{:2d}".format(ttl) + ". " + "{:>15s}".format(addr[0]) + " - " + hostname + " (" + "{:.3f}".format((end-start)*1000) + "ms)")
            break
    else:
        print("[-]" + "{:2d}".format(ttl) + ".           * * * - * * *")

    # termination condition
    ttl += 1
    if ttl > args.max_hops:
        break
print("")