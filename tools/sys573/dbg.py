import socket,json,sys
def cmd(c, port=4370):
    s=socket.create_connection(('127.0.0.1',port),timeout=60)
    s.sendall((json.dumps(c)+'\n').encode())
    data=b''
    while not data.endswith(b'\n'):
        d=s.recv(1<<20)
        if not d: break
        data+=d
    s.close(); return json.loads(data)
if __name__=='__main__':
    print(json.dumps(cmd(json.loads(sys.argv[1])))[:4000])
