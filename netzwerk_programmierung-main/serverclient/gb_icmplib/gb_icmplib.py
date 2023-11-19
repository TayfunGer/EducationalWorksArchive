### 
### ICMP library by Gerland and Baylan
###

### Every values are integers, without data of icmp-package, which is hex as str
### (future prospects: save everything as hex values + exception handling by wrong input)

from email.mime import base
import socket
from enum import IntEnum
from sys import flags


def ms_to_timestamp(int_ms):
    ''' 
        convert a integer of milliseconds to timestamp
        Params: (int) milliseconds
        
        Return: (str) timestamp

    '''
    h = int_ms // 3600000
    m = int_ms % 3600000 // 60000
    s = int_ms % 3600000 % 60000 // 1000
    ms = int_ms % 3600000 % 60000 % 1000
    return "{:02d}".format(h) + ':' + \
           "{:02d}".format(m) + ':' + \
           "{:02d}".format(s) + '.' + \
           "{:03d}".format(ms)


def timestamp_to_ms(str_timestamp):
    ''' 
        convert a string of timestamp to milliseconds
        Params: (str) timestamp

        Return: (int) milliseconds

    '''
    values = str_timestamp.split(':')
    return int(values[0]) * 3600000 + \
           int(values[1]) * 60000 + \
           int(values[2]) * 1000 + \
           int(values[3])


class MessageTypes(IntEnum):
    ECHO_REPLY = 0
    DESTINATION_UNREACHABLE = 3
    SOURCE_QUENCH = 4
    REDIRECT = 5
    ECHO = 8
    TIME_EXCEEDED = 11
    PARAMETER_PROBLEM = 12
    TIMESTAMP = 13
    TIMESTAMP_REPLY = 14
    INFORMATION_REQUEST = 15
    INFORMATION_REPLY = 16


class IPHeader:
    
    ''' 
    class to represent the IP header

    Attributes:
        version
        internet header length
        type of service
        total length
        id
        flags
        fragment offset
        time to live
        protocol
        source
        destination
        header checksum
    '''

    def __init__(self):
        pass

    
    def set_values(self, version, internet_header_length, 
                 type_of_service, total_length, id, flags, 
                 fragment_offset, time_to_live, protocol, 
                 source, destination):
        ''' 
        Sets the individual IP headers composed

        Params:
            version (int): the version of the IP protcol IPV4 = 4 IPV6 = 6
            internet header length (int): length of IP header
            type of service (int):
            total length (int): total length of the bytestream 
            id (int): Number of fragments the packet is split into
            flags (int):
            fragment offset (int): 
            time to live (int):
            protocol (int):
            source ([int]):
            destination ([int]):
            header checksum (int):
        '''
        self.version = version
        self.internet_header_length = internet_header_length
        self.type_of_service = type_of_service
        self.total_length = total_length
        self.id = id
        self.flags = flags
        self.fragment_offset = fragment_offset
        self.time_to_live = time_to_live
        self.protocol = protocol
        self.source = source
        self.destination = destination
        self.header_checksum = self.calc_checksum()

    
    def generate_raw_data(self):
        ''' 
        converts the data from the IP header into a bytestream

        Return: (bytes) bytestream of raw data of the IP package
        '''
        hex_data = str(hex(self.version)[2:].zfill(1))
        hex_data += str(hex(self.internet_header_length)[2:].zfill(1))
        hex_data += str(hex(self.type_of_service)[2:].zfill(2))
        hex_data += str(hex(self.total_length)[2:].zfill(4))
        hex_data += str(hex(self.id)[2:].zfill(4))
        hex_data += str(hex(self.flags*2**13 + self.fragment_offset)[2:].zfill(4))
        hex_data += str(hex(self.time_to_live)[2:].zfill(2))
        hex_data += str(hex(self.protocol)[2:].zfill(2))
        hex_data += str(hex(self.header_checksum)[2:].zfill(4))
        hex_data += str(hex(self.source[0])[2:].zfill(2))
        hex_data += str(hex(self.source[1])[2:].zfill(2))
        hex_data += str(hex(self.source[2])[2:].zfill(2))
        hex_data += str(hex(self.source[3])[2:].zfill(2))
        hex_data += str(hex(self.destination[0])[2:].zfill(2))
        hex_data += str(hex(self.destination[1])[2:].zfill(2))
        hex_data += str(hex(self.destination[2])[2:].zfill(2))
        hex_data += str(hex(self.destination[3])[2:].zfill(2))

        return bytes.fromhex(hex_data)

    
    def load_raw_data(self, raw_data: bytes):
        ''' 
        fills the icmp and the ip header with the incoming raw data
        
        Param:
            raw_data (bytes): incoming data of an ICMP packet   
        '''
        hex_data = raw_data.hex()

        self.version = int(hex_data[0], base=16)
        self.internet_header_length = int(hex_data[1], base=16)
        self.type_of_service = int(hex_data[2:4], base=16)
        self.total_length = int(hex_data[4:8], base=16)
        self.id = int(hex_data[8:12], base=16)
        self.flags = int(bin(int(hex_data[12], base=16))[2:].zfill(4)[0:3], base=2)
        self.fragment_offset = int(bin(int(hex_data[12:16], base=16))[2:].zfill(16)[3:], base=2)
        self.time_to_live = int(hex_data[16:18], base=16)
        self.protocol = int(hex_data[18:20], base=16)
        self.header_checksum = int(hex_data[20:24], base=16)
        self.source = [int(hex_data[24:26], base=16), int(hex_data[26:28], base=16), int(hex_data[28:30], base=16), int(hex_data[30:32], base=16)]
        self.destination = [int(hex_data[32:34], base=16), int(hex_data[34:36], base=16), int(hex_data[36:38], base=16), int(hex_data[38:40], base=16)]

    
    def calc_checksum(self):
        ''' 
        calculate the checksum from raw data for ICMP an IP header
        
        checksum:
            The 16 bit one's complement of the one's complement sum of all 16
            bit words in the header. For computing the checksum, the checksum
            field should be zero.
        '''
        self.header_checksum = 0
        raw_data = self.generate_raw_data()
        header_len = len(raw_data)

        if header_len % 2 != 0:
            raw_data += "00"
            
        i = 0
        while i < header_len:
            highbyte = raw_data[i]
            lowbyte = raw_data[i+1]
            bit_16 = (highbyte << 8) + lowbyte
            self.header_checksum += bit_16
            i = i+2

        self.header_checksum &= 0xffffffff
        self.header_checksum = (self.header_checksum >> 16) + (self.header_checksum & 0xffff)
        self.header_checksum += (self.header_checksum >> 16)
        self.header_checksum = ~self.header_checksum & 0xffff
        return self.header_checksum

    
    def verify_checksum(self):
        ''' 
        checks if the checksum is identical
        
        checksum:
            The 16 bit one's complement of the one's complement sum of all 16
            bit words in the header. For computing the checksum, the checksum
            field should be zero.

        Return: (bool) the status of the verification 
        '''
        checksum = 0
        raw_data = self.generate_raw_data()
        header_len = len(raw_data)

        if header_len % 2 != 0:
            raw_data += "00"

        i = 0
        while i < header_len:
            highbyte = raw_data[i]
            lowbyte = raw_data[i+1]
            bit_16 = (highbyte << 8) + lowbyte
            checksum += bit_16
            i = i+2

        checksum &= 0xffffffff
        checksum = (checksum >> 16) + (checksum & 0xffff)
        checksum += (checksum >> 16)
        checksum = ~checksum & 0xffff
        return checksum == 0

    
    def print_ip_header(self):
        ''' 
        Print the IP header human readable in the terminal 
        '''
        print("<< IP_HEADER >>")
        print("Version         : " + str(self.version))
        print("Header Length   : " + str(self.internet_header_length * 4))
        print("Type of Service : " + str(bin(self.type_of_service)[2:].zfill(8)))
        print("Total Length    : " + str(self.total_length))
        print("Identifier      : " + str(self.id))
        print("Flags           : " + str(bin(self.flags)[2:].zfill(4)[0:3]))
        print("Fragment Offset : " + str(self.fragment_offset))
        print("Time to Live    : " + str(self.time_to_live))
        print("Protocol        : " + str(self.protocol))
        print("Header Checksum : " + str(hex(self.header_checksum)[2:].zfill(4)))
        print("Checksum status : verfied" if self.verify_checksum() else "Checksum status: unverified")
        print("Soruce          : " + str(self.source[0]) + "." + str(self.source[1]) + "." + str(self.source[2]) + "." + str(self.source[3]))
        print("Destination     : " + str(self.destination[0]) + "." + str(self.destination[1]) + "." + str(self.destination[2]) + "." + str(self.destination[3]) + "\n")


class ICMPHeader:
    
    ''' 
    class to represent the ICMP header

    Attributes:
        version
        internet header length
        type of service
        total length
        id
        flags
        fragment offset
        time to live
        protocol
        source
        destination
        header checksum
    '''
    def __init__(self):
        pass

    # done
    def set_values(self, type, code):
        ''' 
        Sets the individual ICMP headers composed

        Params:
            type (int): message type of the ICMP package
            code (int): code of the ICMP package
        '''
        self.type = type
        self.code = code

    # done
    def generate_raw_data(self):
        ''' 
        converts the data from the ICMP header into a bytestream

        Return: (bytes) bytestream of raw data of the ICMP package
        '''
        hex_data = str(hex(self.type)[2:].zfill(2))
        hex_data += str(hex(self.code)[2:].zfill(2))
        hex_data += str(hex(self.checksum)[2:].zfill(4))

        if self.type == MessageTypes.DESTINATION_UNREACHABLE or \
           self.type == MessageTypes.TIME_EXCEEDED or \
           self.type == MessageTypes.SOURCE_QUENCH:
            hex_data += "00000000"
            hex_data += str(hex(self.internet_header)[2:].zfill(self.internet_header_length*2))
            hex_data += str(self.data)
        elif self.type == MessageTypes.PARAMETER_PROBLEM:
            hex_data += str(hex(self.pointer)[2:].zfill(2))
            hex_data += "000000"
            hex_data += str(hex(self.internet_header)[2:].zfill(self.internet_header_length*2))
            hex_data += str(self.data)
        elif self.type == MessageTypes.REDIRECT:
            hex_data += str(hex(self.gateway_internet_address[0])[2:].zfill(2))
            hex_data += str(hex(self.gateway_internet_address[1])[2:].zfill(2))
            hex_data += str(hex(self.gateway_internet_address[2])[2:].zfill(2))
            hex_data += str(hex(self.gateway_internet_address[3])[2:].zfill(2))
            hex_data += str(hex(self.internet_header)[2:].zfill(self.internet_header_length*2))
            hex_data += str(self.data)
        elif self.type == MessageTypes.ECHO or \
             self.type == MessageTypes.ECHO_REPLY:
            hex_data += str(hex(self.id)[2:].zfill(4))
            hex_data += str(hex(self.sequence_number)[2:].zfill(4))
            hex_data += str(self.data)
        elif self.type == MessageTypes.TIMESTAMP or \
             self.type == MessageTypes.TIMESTAMP_REPLY:
            hex_data += str(hex(self.id)[2:].zfill(4))
            hex_data += str(hex(self.sequence_number)[2:].zfill(4))
            hex_data += str(hex(self.originate_timestamp)[2:].zfill(8))
            hex_data += str(hex(self.recive_timestamp)[2:].zfill(8))
            hex_data += str(hex(self.transmit_timestamp)[2:].zfill(8))
        elif self.type == MessageTypes.INFORMATION_REQUEST or \
             self.type == MessageTypes.INFORMATION_REPLY:
            hex_data += str(hex(self.id)[2:].zfill(4))
            hex_data += str(hex(self.sequence_number)[2:].zfill(4))
        
        return bytes.fromhex(hex_data) 

    # done
    def load_raw_data(self, raw_data: bytes):
        ''' 
        fills the icmp and the ip header with the incoming raw data
        
        Param:
            raw_data (bytes): incoming data of an ICMP packet   
        '''
        hex_data = raw_data.hex()
        self.type = int(hex_data[40:42], base=16)
        self.code = int(hex_data[42:44], base=16)
        self.checksum = int(hex_data[44:48], base=16)
        self.internet_header_length = int(hex_data[1], base=16) * 4 # to find end of ip header part

        if self.type == MessageTypes.DESTINATION_UNREACHABLE or \
           self.type == MessageTypes.TIME_EXCEEDED or \
           self.type == MessageTypes.SOURCE_QUENCH: 
            self.internet_header = int(hex_data[56:56+self.internet_header_length*2], base=16)
            self.data = hex_data[56+self.internet_header_length*2:]
        elif self.type == MessageTypes.PARAMETER_PROBLEM:
            self.pointer = int(hex_data[48:50], base=16)
            self.internet_header = int(hex_data[56:56+self.internet_header_length*2], base=16)
            self.data = hex_data[56+self.internet_header_length*2:]
        elif self.type == MessageTypes.REDIRECT:
            self.gateway_internet_address = [int(hex_data[48:50], base=16), int(hex_data[50:52], base=16), \
                                             int(hex_data[52:54], base=16), int(hex_data[54:56], base=16)]
            self.internet_header = int(hex_data[56:56+self.internet_header_length*2], base=16)
            self.data = hex_data[56+self.internet_header_length*2:]
        elif self.type == MessageTypes.ECHO or \
             self.type == MessageTypes.ECHO_REPLY:
            self.id = int(hex_data[48:52], base=16)
            self.sequence_number = int(hex_data[52:56], base=16)
            self.data = hex_data[56:]
        elif self.type == MessageTypes.TIMESTAMP or \
             self.type == MessageTypes.TIMESTAMP_REPLY:
            self.id = int(hex_data[48:52], base=16)
            self.sequence_number = int(hex_data[52:56], base=16)
            self.originate_timestamp = int(hex_data[56:64], base=16)
            self.recive_timestamp = int(hex_data[64:72], base=16)
            self.transmit_timestamp = int(hex_data[72:80], base=16)
        elif self.type == MessageTypes.INFORMATION_REQUEST or \
             self.type == MessageTypes.INFORMATION_REPLY:
            self.id = int(hex_data[48:52], base=16)
            self.sequence_number = int(hex_data[52:56], base=16)
        else:
            print("Loading failed (invalid type)!")

    # done
    def calc_checksum(self):
        ''' 
        calculate the checksum from raw data for ICMP an IP header
        
        checksum:
            The 16 bit one's complement of the one's complement sum of all 16
            bit words in the header. For computing the checksum, the checksum
            field should be zero.
        '''
        self.checksum = 0
        raw_data = self.generate_raw_data()
        header_len = len(raw_data)

        if header_len % 2 != 0:
            raw_data = bytes.fromhex(raw_data.hex() + "00")
            
        i = 0
        while i < header_len:
            highbyte = raw_data[i]
            lowbyte = raw_data[i+1]
            bit_16 = (highbyte << 8) + lowbyte
            self.checksum += bit_16
            i = i+2

        self.checksum &= 0xffffffff
        self.checksum = (self.checksum >> 16) + (self.checksum & 0xffff)
        self.checksum += (self.checksum >> 16)
        self.checksum = ~self.checksum & 0xffff
        return self.checksum

    # done
    def verify_checksum(self):
        ''' 
        checks if the checksum is identical
        
        checksum:
            The 16 bit one's complement of the one's complement sum of all 16
            bit words in the header. For computing the checksum, the checksum
            field should be zero.

        Return: (bool) the status of the verification %
        '''
        checksum = 0
        raw_data = self.generate_raw_data()
        header_len = len(raw_data)

        if header_len % 2 != 0:
            raw_data = bytes.fromhex(raw_data.hex() + "00")

        i = 0
        while i < header_len:
            highbyte = raw_data[i]
            lowbyte = raw_data[i+1]
            bit_16 = (highbyte << 8) + lowbyte
            checksum += bit_16
            i = i+2

        checksum &= 0xffffffff
        checksum = (checksum >> 16) + (checksum & 0xffff)
        checksum += (checksum >> 16)
        checksum = ~checksum & 0xffff
        return checksum == 0

    # done
    def print_icmp_header(self):
        ''' 
        Print the ICMP header human readable in the terminal 
        '''
        print("<< ICMP_HEADER >>")
        print("Type           : " + MessageTypes(self.type).name)
        print("Code           : " + str(self.code))
        print("Checksum       : " + str(hex(self.checksum)[2:].zfill(4)))
        print("Checksum status: verfied" if self.verify_checksum() else "Checksum status: unverified")

        if self.type == MessageTypes.DESTINATION_UNREACHABLE or \
        self.type == MessageTypes.TIME_EXCEEDED or \
        self.type == MessageTypes.SOURCE_QUENCH:
            print("Internet header: " + str(hex(self.internet_header)[2:]))
            format_data = ' '.join(self.data[i:i+2] for i in range(0, len(self.data), 2))
            format_data = "\n\t\t ".join(format_data[i:i+39] for i in range(0, len(format_data), 39))
            print("Data           : " + format_data + '\n')
        elif self.type == MessageTypes.PARAMETER_PROBLEM:
            print("Pointer        : " + str(self.pointer))
            print("Internet header: " + str(hex(self.internet_header)[2:]))
            format_data = ' '.join(self.data[i:i+2] for i in range(0, len(self.data), 2))
            format_data = "\n\t\t ".join(format_data[i:i+39] for i in range(0, len(format_data), 39))
            print("Data           : " + format_data + '\n')
        elif self.type == MessageTypes.REDIRECT:
            print("Gateway Address: " + str(self.gateway_internet_address[0]) + "." + str(self.gateway_internet_address[1]) + "." + str(self.gateway_internet_address[2]) + "." + str(self.gateway_internet_address[3]))
            print("Internet header: " + str(hex(self.internet_header)[2:]))
            format_data = ' '.join(self.data[i:i+2] for i in range(0, len(self.data), 2))
            format_data = "\n\t\t ".join(format_data[i:i+39] for i in range(0, len(format_data), 39))
            print("Data           : " + format_data + '\n')
        elif self.type == MessageTypes.ECHO or self.type == MessageTypes.ECHO_REPLY:
            print("Identifier     : " + str(self.id))
            print("Sequence number: " + str(self.sequence_number))
            format_data = ' '.join(self.data[i:i+2] for i in range(0, len(self.data), 2))
            format_data = "\n\t\t ".join(format_data[i:i+39] for i in range(0, len(format_data), 39))
            print("Data           : " + format_data + '\n')
        elif self.type == MessageTypes.TIMESTAMP or \
            self.type == MessageTypes.TIMESTAMP_REPLY:
            print("Identifier     : " + str(self.id))
            print("Sequence number: " + str(self.sequence_number))
            print("Orginate Time  : " + ms_to_timestamp(self.originate_timestamp)) 
            print("Receive Time   : " + ms_to_timestamp(self.recive_timestamp)) 
            print("Transmit Time  : " + ms_to_timestamp(self.transmit_timestamp)) 
        elif self.type == MessageTypes.INFORMATION_REQUEST or \
            self.type == MessageTypes.INFORMATION_REPLY:
            print("Identifier     : " + str(self.id))
            print("Sequence number: " + str(self.sequence_number))
        else:
            print("Type of ICMP-Package invalid! Skipping ...\n")




#######################
#### Extra classes
#######################

# done
class DestinationUnreachable(ICMPHeader):
    ''' 
        class for ICMP header with Message type Destination unreachable 
    '''
    def __init__(self, code, internet_header, data):
        super().set_values(MessageTypes.DESTINATION_UNREACHABLE, code)
        self.internet_header_length = 20
        self.internet_header = internet_header

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class TimeExceededMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type time exceeded message 
    '''
    def __init__(self, code, internet_header, data):
        super().set_values(MessageTypes.TIME_EXCEEDED, code)
        self.internet_header_length = 20
        self.internet_header = internet_header

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class ParameterProblemMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type parameter problem message 
    '''
    def __init__(self, code, pointer, internet_header, data):
        super().set_values(MessageTypes.PARAMETER_PROBLEM, code)
        self.pointer = pointer
        self.internet_header_length = 20
        self.internet_header = internet_header

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class SourceQuenchMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type source quench message 
    '''
    def __init__(self, internet_header, data):
        super().set_values(MessageTypes.SOURCE_QUENCH, 0)
        self.internet_header_length = 20
        self.internet_header = internet_header

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class RedirectMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type redirect message
    '''
    def __init__(self, code, gateway_internet_address, internet_header, data):
        super().set_values(MessageTypes.REDIRECT, code)
        self.gateway_internet_address = list(map(int, gateway_internet_address.split(".")))
        self.internet_header_length = 20
        self.internet_header = internet_header

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class EchoMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type echo message 
    '''
    def __init__(self, id, sequence_number, data):
        super().set_values(MessageTypes.ECHO, 0)
        self.id = id
        self.sequence_number = sequence_number

        if len(data) % 2 != 0:
            data = "0" + data

        self.data = data
        self.calc_checksum()

# done
class EchoReplyMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type echo reply message
    '''
    def __init__(self, id, sequence_number, data):
        super().set_values(MessageTypes.ECHO_REPLY, 0)
        self.id = id
        self.sequence_number = sequence_number

        if len(data) % 2 != 0:
            data = "0" + data
        
        self.data = data
        self.calc_checksum()

# done
class TimestampMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type time stamp message
    '''
    def __init__(self, id, sequence_number, originate_timestamp,
                 recive_timestamp, transmit_timestamp):
        super().set_values(MessageTypes.TIMESTAMP, 0)
        self.id = id
        self.sequence_number = sequence_number
        self.originate_timestamp = originate_timestamp
        self.recive_timestamp = recive_timestamp
        self.transmit_timestamp = transmit_timestamp
        self.calc_checksum()

# done
class TimestampReplyMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type time stamp reply message
    '''
    def __init__(self, id, sequence_number, originate_timestamp,
                 recive_timestamp, transmit_timestamp):
        super().set_values(MessageTypes.TIMESTAMP_REPLY, 0)
        self.id = id
        self.sequence_number = sequence_number
        self.originate_timestamp = originate_timestamp
        self.recive_timestamp = recive_timestamp
        self.transmit_timestamp = transmit_timestamp
        self.calc_checksum()

# done
class InformationMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type information message
    '''
    def __init__(self, id, sequence_number):
        super().set_values(MessageTypes.INFORMATION_REQUEST, 0)
        self.id = id
        self.sequence_number = sequence_number
        self.calc_checksum()

# done
class InformationReplyMessage(ICMPHeader):
    ''' 
        class for ICMP header with Message type information reply message
    '''
    def __init__(self, id, sequence_number):
        super().set_values(MessageTypes.INFORMATION_REPLY, 0)
        self.id = id
        self.sequence_number = sequence_number
        self.calc_checksum()
