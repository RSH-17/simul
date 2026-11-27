"""Keep Fast DDS multicast membership on the robot's wired interface.

Receives/discards discovery datagrams only; never publishes ROS commands.
Linux DDS sockets sharing this group/port can receive discovery on eth0.
"""
import argparse
import signal
import socket


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='192.168.185.3')
    parser.add_argument('--domain', type=int, default=4)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error('Invalid ROS domain')
    socket.inet_aton(args.address)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        receiver.bind(('', 7400 + 250 * args.domain))
        receiver.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           socket.inet_aton('239.255.0.1') + socket.inet_aton(args.address))
        receiver.settimeout(1.)
        print('Wired DDS discovery membership active: ' + args.address, flush=True)
        while not stopped:
            try:
                receiver.recvfrom(65535)
            except socket.timeout:
                pass


if __name__ == '__main__':
    main()
