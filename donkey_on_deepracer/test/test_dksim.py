# What does the sim say?

import socket

if __name__ == "__main__":
    HOST, PORT = "192.168.0.194", 9091
    cli = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli.connect((HOST, PORT))

    while True:
        data = cli.recv(1024).decode("utf-8")
        print(data)