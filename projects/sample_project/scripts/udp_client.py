import os
import socket
import msgpack

port = 3333
wesp_ip_address = "192.168.98.124" #"172.17.3.174" #"192.168.98.124"
default_payload = {
    "key": 1,
    "another_key": "another_value",
}


# set up UDP server on lico
def set_UDP_sock(address, port):
    for res in socket.getaddrinfo(address, port, socket.AF_UNSPEC,
                                  socket.SOCK_DGRAM, 0, socket.AI_PASSIVE):
        family_addr, _, _, _, addr = res 
    try:
        print( addr)
        sock = socket.socket(family_addr, socket.SOCK_DGRAM)
        sock.settimeout(3000.0)
        print(sock)
    except socket.error as msg:
        print('Could not create socket')
        print(os.strerror(msg.errno))
        raise

    return sock, addr


def send_UDP_packet(sock, addr, payload):
    # incorporate msg pack here 
    try:
        print(payload)
        sock.sendto(payload, addr)
    except socket.timeout:
        print('Socket operation timeout')
        return ''
    except socket.error as msg:
        print('Error while sending or receiving data from the socket')
        print(os.strerror(msg.errno))
        sock.close()
        raise
    return 1


# send UDP packet based on some condition
def main():
    sock, addr = set_UDP_sock(wesp_ip_address, port)
    while True:
        key = input("Key input: ")
        if key == "1":
            payload = "polaris_init".encode()
        elif key == "2":
            payload = "polaris_read".encode()
        elif key == "3":
            payload = msgpack.packb(default_payload, use_bin_type=True)
        else:
            payload = key.encode()
        send_UDP_packet(sock, addr, payload)
    return 0


if __name__ == '__main__':
    main()
