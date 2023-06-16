import os
import socket
import msgpack
import sys

port = 3333
wesp_ip_address = sys.argv[1] or "192.168.98.124"
default_payload = {                     # gpio operation dict
    "gpio": 4,                      # gpio pin to perform on
    "action_seq": [                 # list of action module dicts
        {
            "action": "up",     # up or down
            "duration": 100,    # in ms, -1 for keeping state up/down
            "delay_pre": -1,    # in ms, -1 for no delay before pulse
            "delay_post": -1,   # in ms, -1 for no delay after pulse
            "repeat": 1         # 1 for play action once, 2 for play action twice, etc; -1 for repeat indefinitely 
        },
        {
            "action": "down",
            "duration": 50,
            "delay_pre": 1000,
            "delay_post": -1,
            "repeat": 1
        },
        {
            "action": "up",
            "duration": 100,
            "delay_pre": -1,
            "delay_post": -1,
            "repeat": 2
        },
        {
            "action": "down",
            "duration": 100,
            "delay_pre": -1,
            "delay_post": 500,
            "repeat": 3
        },
        {
            "action": "up",
            "duration": 100,
            "delay_pre": -1,
            "delay_post": -1,
            "repeat": 1
        }

    ],
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
