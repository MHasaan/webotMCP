"""Real-socket regressions for commands whose execution outcome is uncertain."""
import json
import socket
import struct
import threading
import time

import pytest

from connection import BridgeConnection, BridgeError


def exact(sock, count):
    data = b''
    while len(data) < count:
        chunk = sock.recv(count-len(data))
        if not chunk:
            raise ConnectionError('closed')
        data += chunk
    return data


def request(sock):
    size, = struct.unpack('>I', exact(sock, 4))
    return json.loads(exact(sock, size))


def frame(value):
    data = json.dumps(value).encode()
    return struct.pack('>I',len(data))+data


class Peer:
    def __init__(self, handlers):
        self.handlers = handlers
        self.sock = socket.socket()
        self.sock.bind(('127.0.0.1',0))
        self.sock.listen(4)
        self.sock.settimeout(2)
        self.port = self.sock.getsockname()[1]
        self.errors = []
        self.connections = []
        self.thread = threading.Thread(target=self.serve,daemon=True)

    def serve(self):
        try:
            for handler in self.handlers:
                conn,_ = self.sock.accept()
                self.connections.append(conn)
                with conn:
                    conn.settimeout(2)
                    handler(conn)
        except OSError:
            pass  # expected when the timed-out client or fixture closes
        except Exception as exc:
            self.errors.append(exc)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self,*args):
        self.sock.close()
        for conn in self.connections:
            conn.close()
        self.thread.join(3)
        assert not self.thread.is_alive()
        assert not self.errors


def test_executed_command_with_lost_reply_is_never_replayed():
    mutations = []
    def drop(conn):
        r=request(conn)
        mutations.append(r['action'])
        # Operation completed, but the reply was lost.
    def next_connection(conn):
        r=request(conn)
        if r['action']=='increment':
            mutations.append(r['action'])
        conn.sendall(frame({'id':r['id'],'status':'ok','result':len(mutations)}))
    with Peer([drop,next_connection]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            with pytest.raises(BridgeError,match='outcome.*unknown|unknown.*outcome'):
                c.command('increment',timeout=1)
            assert mutations==['increment']
            assert c.command('count',timeout=1)==1
        finally:
            c.close()


def test_heartbeats_do_not_extend_command_deadline():
    def slow(conn):
        r=request(conn)
        for _ in range(20):
            conn.sendall(b'\0\0\0\0')
            time.sleep(.015)
        conn.sendall(frame({'id':r['id'],'status':'ok','result':'late'}))
    with Peer([slow]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            start=time.monotonic()
            with pytest.raises(BridgeError,match='outcome.*unknown|unknown.*outcome'):
                c.command('slow',timeout=.08)
            assert time.monotonic()-start < .25
            assert c.sock is None
        finally:
            c.close()


def test_partial_body_uses_one_deadline_not_one_timeout_per_chunk():
    def drip(conn):
        r=request(conn)
        data=frame({'id':r['id'],'status':'ok','result':'complete'})
        conn.sendall(data[:4])
        for chunk in (data[4:15],data[15:30],data[30:]):
            time.sleep(.04)
            conn.sendall(chunk)
    with Peer([drip]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            with pytest.raises(BridgeError,match='outcome.*unknown|unknown.*outcome'):
                c.command('slow',timeout=.07)
        finally:
            c.close()


def test_mismatched_response_id_is_rejected():
    def wrong(conn):
        r=request(conn)
        conn.sendall(frame({'id':r['id']+1,'status':'ok','result':'wrong command'}))
    with Peer([wrong]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            with pytest.raises(BridgeError,match='outcome.*unknown|unknown.*outcome'):
                c.command('read',timeout=1)
            assert c.sock is None
        finally:
            c.close()


def test_malformed_reply_invalidates_connection():
    def malformed(conn):
        request(conn)
        conn.sendall(b'\0\0\0\1{')
    with Peer([malformed]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            with pytest.raises(BridgeError,match='outcome.*unknown|unknown.*outcome'):
                c.command('read',timeout=1)
            assert c.sock is None
        finally:
            c.close()


def test_fragmented_frame_succeeds_within_deadline():
    def fragmented(conn):
        r=request(conn)
        data=frame({'id':r['id'],'status':'ok','result':{'value':42}})
        for chunk in (data[:2],data[2:7],data[7:]):
            conn.sendall(chunk)
            time.sleep(.005)
    with Peer([fragmented]) as peer:
        c=BridgeConnection(port=peer.port)
        try:
            assert c.command('read',timeout=1)=={'value':42}
        finally:
            c.close()


def test_lock_wait_is_included_in_deadline():
    c = BridgeConnection()
    c.lock.acquire()
    try:
        start = time.monotonic()
        with pytest.raises(BridgeError, match='not sent'):
            c.command('mutate', timeout=.03)
        assert time.monotonic()-start < .2
        assert c.sock is None
    finally:
        c.lock.release()


@pytest.mark.parametrize('timeout', [0, -1, float('nan'), float('inf')])
def test_invalid_timeout_is_rejected_before_connect(timeout):
    c = BridgeConnection()
    with pytest.raises(ValueError, match='timeout'):
        c.command('mutate', timeout=timeout)
    assert c.sock is None


def test_short_agent_timeout_remains_positive():
    def respond(conn):
        r = request(conn)
        assert 0 < r['params']['timeout'] < .5
        conn.sendall(frame({'id':r['id'], 'status':'ok',
                           'result':{'status':'ok','result':{'pong':True}}}))
    with Peer([respond]) as peer:
        c = BridgeConnection(port=peer.port)
        try:
            assert c.robot_command('demo','ping',timeout=.5) == {'pong':True}
        finally:
            c.close()


def test_valid_error_does_not_poison_next_response():
    def respond(conn):
        r = request(conn)
        conn.sendall(frame({'id':r['id'],'status':'error','error':'missing node'}))
        r = request(conn)
        conn.sendall(frame({'id':r['id'],'status':'ok','result':42}))
    with Peer([respond]) as peer:
        c = BridgeConnection(port=peer.port)
        try:
            with pytest.raises(BridgeError, match='missing node'):
                c.command('get_missing', timeout=1)
            assert c.command('read', timeout=1) == 42
        finally:
            c.close()
