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
parser.add_argument("-a", "--address", dest="address", default="127.0.0.1", type=str)
parser.add_argument("-c", "--count", dest="count", default="5", type=int)
parser.add_argument("-i", "--interval", dest="interval", default=1, type=float)
parser.add_argument("-d", "--data", dest="data", default=default_message, type=str)
args = parser.parse_args()

raw_sock = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)

print("\nPinging " + args.address)
id = randint(0, 65535)
sent = 0
recived = 0
times = []

for i in range(1, args.count+1):
      count = 0
      sent += 1
      raw_data = icmp.EchoMessage(id, i, hex(int(args.data, base=16))[2:]).generate_raw_data()
      start = time()
      raw_sock.sendto(raw_data, (args.address, 0))
      while time() < start + args.interval:
            ready = select.select([raw_sock], [], [], start - time() + args.interval)
            if ready[0]:
                  data, addr = raw_sock.recvfrom(1024)
                  end = time()
            else: 
                  break

            ip_header = icmp.IPHeader()
            icmp_header = icmp.ICMPHeader()
            ip_header.load_raw_data(data)
            icmp_header.load_raw_data(data)

            if icmp_header.type == icmp.MessageTypes.ECHO_REPLY and \
               icmp_header.id == id and \
               icmp_header.sequence_number == i:
                  count += 1
                  if count <= 1:
                        recived += 1
                        times.append(end-start)

                  header_len = len(icmp_header.generate_raw_data().hex()) // 2

                  print(str(header_len) + " Bytes from " + addr[0] + \
                        ": icmp_seq=" + str(icmp_header.sequence_number) + \
                        " ttl=" + str(ip_header.time_to_live) + \
                        " time=" + "{:.3f}ms".format((end-start)*1000) + \
                        (" (Dup!)" if count != 1 else ""))

# division by zero
packet_loss = (1-sent/recived)*100 if recived else 100
average_time = sum(times) / len(times) * 1000 if recived else 0
min_time = min(times) * 1000 if recived else 0
max_time = max(times) * 1000 if recived else 0

print("\nPackets sent:     " + str(sent) + "\nPackets recived:  " + str(recived) + \
      "\nPacket loss:      " + "{:.2f}%".format(packet_loss) + \
      "\nMin/Max/Avg time: " + "{:.3f}/{:.3f}/{:.3f} ms\n".format(min_time, max_time, average_time))