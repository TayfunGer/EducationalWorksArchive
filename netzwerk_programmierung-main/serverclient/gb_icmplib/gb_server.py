import socket
import gb_icmplib as icmp
from os import system
from time import localtime, strftime
from datetime import datetime

# Start
system("clear")
print("""
 ___ ____ __  __ ____    ____                           \n\
|_ _/ ___|  \/  |  _ \  / ___|  ___ _ ____   _____ _ __ \n\
 | | |   | |\/| | |_) | \___ \ / _ \ '__\ \ / / _ \ '__|\n\
 | | |___| |  | |  __/   ___) |  __/ |   \ V /  __/ |   \n\
|___\____|_|  |_|_|     |____/ \___|_|    \_/ \___|_|   \n""") 

print("********************************************************\n")

# create raw_socket to recive ICMP packages
raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)

while True:
    # receive data with buffersize 1024
    data, addr = raw_sock.recvfrom(1024)

    ip_header = icmp.IPHeader()
    ip_header.load_raw_data(data)
    icmp_header = icmp.ICMPHeader()
    icmp_header.load_raw_data(data)

    print("New packet at " + strftime("%H:%M:%S\n", localtime()))
    ip_header.print_ip_header()
    icmp_header.print_icmp_header()

    print("********************************************************\n")

    
    if icmp_header.type == icmp.MessageTypes.ECHO:
        reply = icmp.EchoReplyMessage(icmp_header.id, icmp_header.sequence_number, icmp_header.data)
        raw_sock.sendto(reply.generate_raw_data(), addr)
    elif icmp_header.type == icmp.MessageTypes.TIMESTAMP:
        now = datetime.utcnow().strftime("%H:%M:%S:%f")[:-3]
        reply = icmp.TimestampReplyMessage(icmp_header.id, icmp_header.sequence_number, icmp_header.originate_timestamp, \
                                           icmp.timestamp_to_ms(now), icmp.timestamp_to_ms(now))
        raw_sock.sendto(reply.generate_raw_data(), addr)
    elif icmp_header.type == icmp.MessageTypes.INFORMATION_REQUEST:
        reply = icmp.InformationReplyMessage(icmp_header.id, icmp_header.sequence_number)
        raw_sock.sendto(reply.generate_raw_data(), addr)
