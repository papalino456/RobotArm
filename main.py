import socket

def send_angles(angles):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(('192.168.100.121', 12345))
        s.sendall(bytes(','.join(map(str, angles)), 'utf-8'))
        data = s.recv(1024)
    print('Received', repr(data.decode('utf-8')))

angles = [90, 90, 120, 0, 0]  # Example angles
send_angles(angles)