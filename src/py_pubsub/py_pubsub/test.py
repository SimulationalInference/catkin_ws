import threading
import queue
import time

q = queue.Queue()
class send(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        print("OK")
        time.sleep(1)
        q.put(1)

class recv(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        print("OK")
        a = q.get(block=True, timeout=10000)
        print(a)
        
a = send()
b = recv()

a.start()
b.start()
a.join()
b.join()