#!/usr/bin/env python3
"""Find Kutum on the network, from anywhere: the host, a container, a laptop.

    kutum/find.py                 print Kutum's address (exit 1 if not found)
    kutum/find.py --proxy 22      connect to it and relay stdin/stdout; for
                                  ssh's ProxyCommand (kutum/ssh_config)

Why this exists. `kutum.local` is answered by multicast DNS, and the usual
resolver path (nss-mdns -> the avahi daemon's socket) breaks in a container
whose /run does not hold a live avahi socket, or on a machine with no avahi at
all: the name then fails while Kutum is right there on the LAN. So this asks
the network itself, in order, and stops at the first answer that takes a TCP
connection on port 22:

  1. a multicast DNS query for `<name>.local`, sent directly (no daemon);
  2. the system resolver, with a short timeout;
  3. an IPv6 link-local address the resolver or the neighbour table knows,
     with the interface zone added (fe80:: is useless without one);
  4. the last address that worked, cached in ~/.cache/mongla/kutum-addr;
  5. a sweep of each local /24 for port 22, keeping the address whose MAC is
     Kutum's (one per line in ~/.config/mongla/kutum.macs, or $KUTUM_MACS).
     This one needs no name service and no multicast at all: a Wi-Fi network
     that drops multicast between clients (measured here, 2026-11-21: no mDNS
     answer by IPv4 or IPv6 while ssh answered) still finds it in about 1 s.

Standard library only, so it runs before anything is installed.
"""
from __future__ import annotations

import argparse
import os
import random
import select
import socket
import struct
import subprocess
import sys
import threading
import time

NAME = os.environ.get('KUTUM_NAME', 'kutum')
CACHE = os.path.expanduser('~/.cache/mongla/kutum-addr')
MDNS = ('224.0.0.251', 5353)
MACS_FILE = os.path.expanduser('~/.config/mongla/kutum.macs')


def _macs() -> set[str]:
    raw = os.environ.get('KUTUM_MACS', '')
    try:
        with open(MACS_FILE) as f:
            raw += ' ' + f.read()
    except OSError:
        pass
    return {m.lower() for m in raw.replace(',', ' ').split() if m.count(':') == 5}


def _lan_prefixes() -> list[str]:
    """'192.168.0.' for every up IPv4 /24 that is not a container or VPN bridge."""
    try:
        r = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'up'], capture_output=True,
                           text=True, timeout=2.0)
    except (OSError, subprocess.TimeoutExpired):
        return []
    out = []
    for line in r.stdout.splitlines():
        f = line.split()
        if len(f) < 4 or f[1] == 'lo' or f[1].startswith(('docker', 'br-', 'veth', 'tailscale')):
            continue
        ip, _, plen = f[3].partition('/')
        if plen and int(plen) >= 16:
            out.append(ip.rsplit('.', 1)[0] + '.')
    return out


def sweep(macs: set[str], port: int = 22) -> list[str]:
    """Addresses on the local /24s with `port` open whose MAC is in `macs`."""
    if not macs:
        return []
    from concurrent.futures import ThreadPoolExecutor
    hosts = [f'{p}{i}' for p in _lan_prefixes() for i in range(1, 255)]
    with ThreadPoolExecutor(64) as ex:
        open_ = [h for h, ok in zip(hosts, ex.map(lambda h: _answers_ssh(h, port, 0.6), hosts))
                 if ok]
    try:
        r = subprocess.run(['ip', '-4', 'neigh'], capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.TimeoutExpired):
        return []
    mac_of = {f[0]: f[f.index('lladdr') + 1].lower()
              for f in (ln.split() for ln in r.stdout.splitlines()) if 'lladdr' in f}
    return [h for h in open_ if mac_of.get(h) in macs]


def _qname(name: str) -> bytes:
    return b''.join(bytes([len(p)]) + p.encode() for p in name.split('.')) + b'\0'


def _skip_name(buf: bytes, i: int) -> int:
    while True:
        n = buf[i]
        if n == 0:
            return i + 1
        if n & 0xC0 == 0xC0:
            return i + 2
        i += n + 1


def mdns_a(name: str, timeout: float = 1.0) -> list[str]:
    """IPv4 addresses for `name` (e.g. 'kutum.local') by one mDNS query."""
    qid = random.randrange(1 << 16)
    # QU bit (0x8000 on the class) asks for a unicast reply straight to us.
    pkt = struct.pack('!HHHHHH', qid, 0, 1, 0, 0, 0) + _qname(name) + struct.pack('!HH', 1, 0x8001)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
    out: list[str] = []
    try:
        s.bind(('', 0))
        s.sendto(pkt, MDNS)
        end = time.monotonic() + timeout
        while not out and time.monotonic() < end:
            r, _, _ = select.select([s], [], [], max(0.0, end - time.monotonic()))
            if not r:
                break
            buf, _ = s.recvfrom(4096)
            out += _answers_a(buf, name)
    except OSError:
        pass
    finally:
        s.close()
    return out


def _answers_a(buf: bytes, name: str) -> list[str]:
    try:
        _, flags, qd, an, ns, ar = struct.unpack('!HHHHHH', buf[:12])
        i = 12
        for _ in range(qd):
            i = _skip_name(buf, i) + 4
        got = []
        for _ in range(an + ns + ar):
            i = _skip_name(buf, i)
            typ, _cls, _ttl, rdlen = struct.unpack('!HHIH', buf[i:i + 10])
            i += 10
            if typ == 1 and rdlen == 4:
                got.append(socket.inet_ntoa(buf[i:i + 4]))
            i += rdlen
        return got
    except (struct.error, IndexError):
        return []


def _resolver(name: str, timeout: float = 2.0) -> list[str]:
    """getent, bounded: a hung nss module must not hang the caller."""
    try:
        r = subprocess.run(['getent', 'ahosts', name], capture_output=True,
                           text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return []
    seen = []
    for line in r.stdout.splitlines():
        f = line.split()
        a = f[0] if f else ''
        if a and a not in seen:
            seen.append(a)
    return seen


def _zoned(addr: str) -> str:
    """fe80::... -> fe80::...%iface, from the neighbour table."""
    if not addr.lower().startswith('fe80') or '%' in addr:
        return addr
    try:
        r = subprocess.run(['ip', '-6', 'neigh', 'show', addr], capture_output=True,
                           text=True, timeout=2.0)
        f = r.stdout.split()
        if 'dev' in f:
            return f'{addr}%{f[f.index("dev") + 1]}'
    except (OSError, subprocess.TimeoutExpired):
        pass
    return addr


def _answers_ssh(addr: str, port: int = 22, timeout: float = 1.5) -> bool:
    try:
        info = socket.getaddrinfo(addr, port, type=socket.SOCK_STREAM)[0]
        with socket.socket(info[0], socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect(info[4])
        return True
    except OSError:
        return False


def _cached() -> list[str]:
    try:
        with open(CACHE) as f:
            c = f.read().strip()
        return [c] if c else []
    except OSError:
        return []


def candidates(name: str = NAME) -> list[str]:
    """Name-service answers: mDNS, then the system resolver (asked once)."""
    fq = name if '.' in name else f'{name}.local'
    res = _resolver(fq)
    out: list[str] = []
    for a in mdns_a(fq) + [a for a in res if ':' not in a]:
        if a not in out:
            out.append(a)
    for a in res:
        z = _zoned(a) if ':' in a else ''
        if '%' in z and z not in out:
            out.append(z)
    return out


def find(name: str = NAME, port: int = 22) -> str | None:
    # The cache first: on a working network it answers in one connect. Each
    # later source is tried only when everything before it failed.
    tried = set()
    for source in (_cached, lambda: candidates(name), lambda: sweep(_macs(), port)):
        for a in source():
            if a in tried:
                continue
            tried.add(a)
            if not _answers_ssh(a, port):
                continue
            try:
                os.makedirs(os.path.dirname(CACHE), exist_ok=True)
                with open(CACHE, 'w') as f:
                    f.write(a + '\n')
            except OSError:
                pass
            return a
    return None


def _proxy(addr: str, port: int) -> int:
    info = socket.getaddrinfo(addr, port, type=socket.SOCK_STREAM)[0]
    s = socket.socket(info[0], socket.SOCK_STREAM)
    s.connect(info[4])
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def up():
        fd = sys.stdin.fileno()
        while True:
            b = os.read(fd, 65536)
            if not b:
                s.shutdown(socket.SHUT_WR)
                return
            s.sendall(b)
    threading.Thread(target=up, daemon=True).start()
    out = sys.stdout.fileno()
    while True:
        b = s.recv(65536)
        if not b:
            return 0
        os.write(out, b)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--name', default=NAME)
    ap.add_argument('--proxy', type=int, metavar='PORT',
                    help='relay stdin/stdout to this port on Kutum (ssh ProxyCommand)')
    ap.add_argument('--all', action='store_true', help='list every candidate address')
    a = ap.parse_args(argv)
    if a.all:
        for c in _cached() + candidates(a.name) + sweep(_macs()):
            print(c, 'ssh' if _answers_ssh(c) else 'no answer')
        return 0
    addr = find(a.name, a.proxy or 22)
    if addr is None:
        print(f'{a.name}: not found (mDNS, resolver, link-local, cache). Is it on '
              f'this LAN? kutum/README.md, "Finding Kutum".', file=sys.stderr)
        return 1
    if a.proxy:
        return _proxy(addr, a.proxy)
    print(addr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
