from zeroconf import ServiceBrowser, Zeroconf
import socket
import time

class MyListener:
    def __init__(self):
        self.camera_ips = {}

    def add_service(self, zeroconf, type, name):
        info = zeroconf.get_service_info(type, name)
        if info and name.startswith("camera"):
            address = socket.inet_ntoa(info.addresses[0])
            print(f"Found camera at {address} with name {name}")
            self.camera_ips[name] = address

zeroconf = Zeroconf()
listener = MyListener()
browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)

print("Searching for cameras...")
time.sleep(5)  # Wait for a few seconds to find all cameras

# Now you can connect to the cameras using the discovered IPs
for name, ip in listener.camera_ips.items():
    print(f"Connecting to {name} at {ip}")
    # Make a request to the /data endpoint
    # For example: requests.get(f"http://{ip}/data")

zeroconf.close()
