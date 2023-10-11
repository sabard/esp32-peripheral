import os
import socket
import msgpack
import sys

port = 3333

if len(sys.argv) > 1:
    wesp_ip_address = sys.argv[1] 
else:
    wesp_ip_address = "192.168.138.40"

gpio_payload_4 = {                     # gpio operation dict
    "gpio": 4,                      # gpio pin to perform on
    "action_seq": [                 # list of action module dicts
        {
            "action": "up",     # up or down
            "duration": 300,    # in ms, -1 for keeping state up/down
            "delay_pre": 150,    # in ms, -1 for no delay before pulse
            "delay_post": 150,   # in ms, -1 for no delay after pulse
            "repeat": -1,         # 1 for play action once, 2 for play action twice, etc; -1 for repeat indefinitely 
        },
    ],
}
gpio_payload_15 = {                     # gpio operation dict
    "gpio": 15,                      # gpio pin to perform on
    "action_seq": [                 # list of action module dicts
        {
            "action": "up",     # up or down
            "duration": 2000,    # in ms, -1 for keeping state up/down
            "delay_pre": 150,    # in ms, -1 for no delay before pulse
            "delay_post": 150,   # in ms, -1 for no delay after pulse
            "repeat": 1,         # 1 for play action once, 2 for play action twice, etc; -1 for repeat indefinitely 
        },
    ],
}
gpio_payload_13 = {                     # gpio operation dict
    "gpio": 13,                      # gpio pin to perform on
    "action_seq": [                 # list of action module dicts
        {
            "action": "up",     # up or down
            "duration": 1000,    # in ms, -1 for keeping state up/down
            "delay_pre": -1,    # in ms, -1 for no delay before pulse
            "delay_post": -1,   # in ms, -1 for no delay after pulse
            "repeat": -1,         # 1 for play action once, 2 for play action twice, etc; -1 for repeat indefinitely 
        },
    ],
}

ledc_payload_4 = {                     # gpio operation dict
    "ledc": 4,                      # gpio pin to perform on
    "freq": 70,     # in hz
    "duty": 50,    # percent 
}

b_payload = {                     # gpio operation dict
    "gpio": 4,                      # gpio pin to perform on
    "action_seq": [                 # list of action module dicts
        {
            "action": "up",     # up or down
            "duration": 1,    # in ms, -1 for keeping state up/down
            "delay_pre": 1,    # in ms, -1 for no delay before pulse
            "delay_post": 6,   # in ms, -1 for no delay after pulse
            "repeat": -1,         # 1 for play action once, 2 for play action twice, etc; -1 for repeat indefinitely 
        },

    ],
}

other_payload = {}

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
            payload = "polaris_stream".encode()
        elif key == "3":
            payload = "polaris_down".encode()
        elif key == "g4":
            print(gpio_payload_4)
            payload = msgpack.packb(gpio_payload_4, use_bin_type=True)
        elif key == "g15":
            print(gpio_payload_15)
            payload = msgpack.packb(gpio_payload_15, use_bin_type=True)
        elif key == "g13":
            print(gpio_payload_13)
            payload = msgpack.packb(gpio_payload_13, use_bin_type=True)
        elif key == "l4":
            payload = msgpack.packb(ledc_payload_4, use_bin_type=True)
        elif key == "k":
            task_to_kill = input("kill task number:")
            other_payload.update({"killtasknum": int(task_to_kill)})
            print(other_payload)
            payload = msgpack.packb(other_payload, use_bin_type=True)
        else:
            payload = key.encode()
        send_UDP_packet(sock, addr, payload)
    return 0


if __name__ == '__main__':
    main()

#test
