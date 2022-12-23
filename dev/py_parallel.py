import multiprocessing as mp
import time


class Main:
    def __init__(self):
        self.count = 0

    # add 1000000 to count asynchronusly using threads
    def add_async(self):
        for i in range(10000000):
            self.count += 1

    def print_update(self):
        print(self.count)

    def block(self):
        y = 0
        for i in range(10000000):
            y += 1


m = Main()
m.add_async()
m.print_update()
m.add_async()
m.print_update()
# t = threading.Thread(target=m.add_async)
# t.start()
# while t.is_alive():
#     m.print_update()
#     time.sleep(1)
#     m.block()
#     print(m.count)
