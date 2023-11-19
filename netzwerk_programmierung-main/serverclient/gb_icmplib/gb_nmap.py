from random import randint
import gb_icmplib as icmp
import socket
import select
from argparse import ArgumentParser
from time import sleep, time, localtime, strftime

DEFAULT_UDP_PROBE_PORT_SPEC = 40125
PORTS = [21, 22, 23, 25, 53, 68, 69, 80, 110, 111, 123, 135, 137, 139, 143, 161, 389, 443, 445, 587, 636, \
         1080, 1433, 2049, 3306]
SERVICES = ["ftp", "ssh", "telnet", "smtp", "domain", "bootpc", "tftp", "http", "pop3", "sunrpc", "ntp", \
            "loc-srv/epmap", "netbios-ns", "netbios-ssn", "imap", "snmp", "ldap", "https", "microsoft-ds", \
            "submission", "ldaps", "socks", "ms-sql-s", "nfs", "mysql"] 

# returns standard service of port
def get_service_name(port):
    try:
        return SERVICES[PORTS.index(port)]
    except ValueError:
        return "---"

# Check argv to make it optional with default values
parser = ArgumentParser()
parser.add_argument("-a", "--address", dest="address", default="localhost", type=str)
parser.add_argument("-p", "--port", dest="ports", default=None, type=str)
parser.add_argument("-i", "--interval", dest="interval", default=0.5, type=float)
parser.add_argument("-s", "--scan", dest="scan", default="T", type=str)
parser.add_argument("-P", "--ping", dest="ping", default="E", type=str)
args = parser.parse_args()

# 0. title
print(strftime("\n<< Start gb_nmap at %H:%M:%S >>", localtime()))

# 1. host discovery
# udp 
if args.ping == 'U':
    # sockets
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    raw_sock.setblocking(0)

    icmp_header = icmp.ICMPHeader()

    # ping
    start = time()
    udp_sock.sendto(str.encode(""), (args.address, DEFAULT_UDP_PROBE_PORT_SPEC))
    while time() < start + args.interval:
            ready = select.select([raw_sock], [], [], start - time() + args.interval)
            if ready[0]:
                data, addr = raw_sock.recvfrom(1024)
                end = time()
            else:
                print("Note: Host is down or blocking the ping probes.")
                break

            icmp_header.load_raw_data(data)
            if icmp_header.type == icmp.MessageTypes.DESTINATION_UNREACHABLE:
                print("Host is up ({:.2f}s latency)".format(end-start))
                break

# echo request
elif args.ping == 'E':
    tmp_id = randint(0, 65535)
    raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    raw_sock.setblocking(0)
    echo = icmp.EchoMessage(tmp_id, 1, "48656c6c6f21") # send hello
    icmp_header = icmp.ICMPHeader()
    
    start = time()
    raw_sock.sendto(echo.generate_raw_data(), (args.address, 0))
    while time() < start + args.interval:
            ready = select.select([raw_sock], [], [], start - time() + args.interval)
            if ready[0]:
                data, addr = raw_sock.recvfrom(1024)
                end = time()
            else:
                print("Note: Host is down or blocking the ping probes.")
                break

            icmp_header.load_raw_data(data)
            if icmp_header.type == icmp.MessageTypes.ECHO_REPLY and \
               icmp_header.id == tmp_id and icmp_header.sequence_number == 1:
                print("Host is up ({:.2f}s latency)".format(end-start))
                break

# tcp
#elif args.ping == 'T':

else:
    print("Ping method not available. Skipping ...")

# 3. generate portlist
if not args.ports:
    if args.scan == 'U':
        args.ports = "53,68,69,111,123,137,161,389,636,1194,1900,2049,5353,11211"
    elif args.scan == 'T':
        args.ports = "21,22,23,25,53,80,110,111,135,139,143,389,443,445,587,1025,1080,1433,2049,3306,3389,5900,6001,6379,8080"

# 4. portscan
print("\nPORT      STATE           SERVICE")

# udp scan
if args.scan[0] == 'U':
    # sockets
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    udp_sock.sendto(str.encode(""), (args.address, DEFAULT_UDP_PROBE_PORT_SPEC))
    raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    raw_sock.setblocking(0)

    # testing all ports
    for port in sorted([int(p) for p in args.ports.split(',')]):
        print((str(port)+"/udp").ljust(10, ' '), end="")
        start = time()
        udp_sock.sendto(str.encode(""), (args.address, port))
        while time() < start + args.interval:
            ready = select.select([raw_sock], [], [], start - time() + args.interval)
            if ready[0]:
                data, addr = raw_sock.recvfrom(1024)
            else:
                print("open | filtered".ljust(16, ' '), end="")
                break

            icmp_header.load_raw_data(data)
            if icmp_header.type == icmp.MessageTypes.DESTINATION_UNREACHABLE:
                print("closed".ljust(16, ' '), end="")
                break
        print(get_service_name(port))
        try:
            sleep(start - time() + args.interval)
        except Exception:
            continue

# tcp scan
elif args.scan[0] == 'T':
    for port in sorted([int(p) for p in args.ports.split(',')]):
        print((str(port)+"/tcp").ljust(10, ' '), end="")
        start = time()
        try:
            tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM, socket.IPPROTO_TCP)
            tcp_sock.settimeout(args.interval)
            tcp_sock.connect((args.address, port))
        except socket.timeout:
            print("filtered".ljust(16, ' '), end="")
            print(get_service_name(port))
            continue
        except Exception:
            print("closed".ljust(16, ' '), end="")
            print(get_service_name(port))
            continue
        while time() < start + args.interval:
            ready = select.select([raw_sock], [], [], start - time() + args.interval)
            if ready[0]:
                data, addr = raw_sock.recvfrom(1024)
            else:
                print("open".ljust(16, ' '), end="")
                break

            icmp_header.load_raw_data(data)
            if icmp_header.type == icmp.MessageTypes.DESTINATION_UNREACHABLE:
                print("closed".ljust(16, ' '), end="")
        print(get_service_name(port))
        tcp_sock.close()

else:
    print("Scan method not available")

print("")