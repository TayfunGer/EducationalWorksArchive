import socket
import gb_icmplib as icmp
import select
from os import system

system("clear")
print("""                          
 ___ ____ __  __ ____     ____ _ _            _   \n\
|_ _/ ___|  \/  |  _ \   / ___| (_) ___ _ __ | |_ \n\
 | | |   | |\/| | |_) | | |   | | |/ _ \ '_ \| __|\n\
 | | |___| |  | |  __/  | |___| | |  __/ | | | |_ \n\
|___\____|_|  |_|_|      \____|_|_|\___|_| |_|\__|\n""") 

print("********************************************************\n")

print("<< Types >>")
print("""ECHO_REPLY = 0
DESTINATION_UNREACHABLE = 3
SOURCE_QUENCH = 4
REDIRECT = 5
ECHO = 8
TIME_EXCEEDED = 11
PARAMETER_PROBLEM = 12
TIMESTAMP = 13
TIMESTAMP_REPLY = 14
INFORMATION_REQUEST = 15
INFORMATION_REPLY = 16\n""")

print("Create your ICMP-Package:")

type = int(input("Type            = "))

if type == icmp.MessageTypes.DESTINATION_UNREACHABLE:
    code = int(input("Code            = "))
    ih = int(input("Internet header = "), base=16)
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.DestinationUnreachable(code, ih, data)
elif type == icmp.MessageTypes.TIME_EXCEEDED:
    code = int(input("Code            = "))
    ih = int(input("Internet header = "), base=16)
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.TimeExceededMessage(code, ih, data)
elif type == icmp.MessageTypes.PARAMETER_PROBLEM:
    code = int(input("Code            = "))
    pointer = int(input("Pointer         = "))
    ih = int(input("Internet header = "), base=16)
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.ParameterProblemMessage(code, pointer, ih, data)
elif type == icmp.MessageTypes.SOURCE_QUENCH:
    ih = int(input("Internet header = "), base=16)
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.SourceQuenchMessage(ih, data)
elif type == icmp.MessageTypes.REDIRECT:
    code = int(input("Code            = "))
    gateway = input("Gateway I.A.    = ")
    ih = int(input("Internet header = "), base=16)
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.RedirectMessage(code, gateway, ih, data)
elif type == icmp.MessageTypes.ECHO:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.EchoMessage(id, sn, data)
elif type == icmp.MessageTypes.ECHO_REPLY:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    data = hex(int(input("Data            = "), base=16))[2:]
    icmp_header = icmp.EchoReplyMessage(id, sn, data)
elif type == icmp.MessageTypes.TIMESTAMP:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    ot = int(input("Originate TS    = "))
    rt = int(input("Receive TS      = "))
    tt = int(input("Transmit T.s.   = "))
    icmp_header = icmp.TimestampMessage(id, sn, ot, rt, tt)
elif type == icmp.MessageTypes.TIMESTAMP_REPLY:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    ot = int(input("Originate TS    = "))
    rt = int(input("Receive TS      = "))
    tt = int(input("Transmit TS     = "))
    icmp_header = icmp.TimestampReplyMessage(id, sn, ot, rt, tt)
elif type == icmp.MessageTypes.INFORMATION_REQUEST:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    icmp_header = icmp.InformationMessage(id, sn)
elif type == icmp.MessageTypes.INFORMATION_REPLY:
    id = int(input("Identifier      = "))
    sn = int(input("Sequence number = "))
    icmp_header = icmp.InformationReplyMessage(id, sn)
else:
    print("Wrong type. Exiting ...\n")
    exit()

print("")
icmp_header.print_icmp_header()
choice = input("Do you wanna send this package (y/n)? ")

if choice[0] == 'y':
    addr = input("Address = ")
    raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
    raw_sock.sendto(icmp_header.generate_raw_data(), (addr, 0))
    print("Package sent. ", end="")

    if icmp_header.type == icmp.MessageTypes.ECHO or \
       icmp_header.type == icmp.MessageTypes.INFORMATION_REQUEST or \
       icmp_header.type == icmp.MessageTypes.TIMESTAMP:
        print("Waiting for reply ...\n")

        while True:
            ready = select.select([raw_sock], [], [], 1)
            if ready[0]:
                data, addr = raw_sock.recvfrom(1024)
            else:
                print("No reply. Exit ...")
                break

            reply = icmp.ICMPHeader()
            reply.load_raw_data(data)
            if (reply.type == icmp.MessageTypes.ECHO_REPLY or \
               reply.type == icmp.MessageTypes.TIMESTAMP_REPLY or \
               reply.type == icmp.MessageTypes.INFORMATION_REPLY) and \
               reply.id == icmp_header.id and \
               reply.sequence_number == icmp_header.sequence_number:
                reply.print_icmp_header()
                break
    else:
        print("Exit ...\n")
else:
    print("Client ended ...\n")