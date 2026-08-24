#!/usr/bin/env python3
"""Relay Docker Desktop TCP X11 clients to the local WSLg X11 socket."""

import socket
import threading


LISTEN = ("0.0.0.0", 6000)
X11_SOCKET = "/tmp/.X11-unix/X0"


def relay(client: socket.socket) -> None:
    upstream = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    upstream.connect(X11_SOCKET)

    def copy(source: socket.socket, target: socket.socket) -> None:
        try:
            while data := source.recv(65536):
                target.sendall(data)
        except (ConnectionError, OSError):
            pass
        finally:
            try:
                target.shutdown(socket.SHUT_WR)
            except OSError:
                pass

    threads = (
        threading.Thread(target=copy, args=(client, upstream), daemon=True),
        threading.Thread(target=copy, args=(upstream, client), daemon=True),
    )
    try:
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
    finally:
        client.close()
        upstream.close()


def main() -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(LISTEN)
    server.listen(8)
    while True:
        client, _ = server.accept()
        threading.Thread(target=relay, args=(client,), daemon=True).start()


if __name__ == "__main__":
    main()
