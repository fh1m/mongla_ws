"""Where a tool's web page can actually be reached, resolved at RUN time.

⛔ NEVER PRINT A HARDCODED IP. Five tools in this directory printed
`http://10.42.0.28:<port>`, which is the Pi's address on one particular
NetworkManager shared link and changes whenever the link is rebuilt or the
vehicle is plugged into a venue network. An operator following a printed URL
that quietly stopped being true reads it as "the server did not start".

The mDNS name is the stable one -- the Pi answers to `mongla.local` at every
address it has ever had -- so it goes FIRST, with the live addresses listed
after it as the fallback for a network where mDNS is filtered.

Standalone and dependency-free on purpose: `tools/` is loaded without a ROS
environment (importing `duburi_vision` drags in `rclpy`), so this cannot
live in the package.
"""
import socket
import subprocess


def hostnames():
    """The names and addresses this machine can be reached at, best first."""
    out = []
    host = socket.gethostname().split('.')[0]
    if host:
        out.append(f'{host}.local')
    try:
        # `hostname -I` lists only real, up, non-loopback addresses -- which
        # is what an operator can actually type. Parsing `ip addr` would also
        # hand back link-local and docker bridges.
        for ip in subprocess.run(['hostname', '-I'], capture_output=True,
                                 text=True, timeout=2).stdout.split():
            # Skip loopback, IPv6, and the docker/podman bridges -- an
            # operator cannot reach the vehicle on 172.17.0.1 and a wrong
            # URL in the list is worse than a shorter list.
            if (':' in ip or ip.startswith('127.')
                    or ip.startswith('169.254.')
                    or 172 == int(ip.split('.')[0])
                    and 16 <= int(ip.split('.')[1]) <= 31):
                continue
            out.append(ip)
    except Exception:
        pass
    return out or ['localhost']


def where(port):
    """One line naming every URL that reaches `port` on this machine."""
    return '  open  ' + '   or   '.join(f'http://{h}:{port}'
                                        for h in hostnames())


if __name__ == '__main__':
    import sys
    print(where(int(sys.argv[1]) if len(sys.argv) > 1 else 8099))
