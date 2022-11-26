def get_degree():  # 0-100
    return 0


def get_distance():
    return 0


def publish(arr):
    return 0


arr = []
where_in_middle = False
RPM = 2
HZ = 1000
MIN_LENGTH = HZ / RPM * 0.9

while True:
    degree = get_degree()
    if degree >= 80 and degree <= 90:
        where_in_middle = True
    elif where_in_middle and degree >= 20 and degree <= 30 and len(arr) > MIN_LENGTH:
        publish(arr)
        where_in_middle = False
        arr = []
    arr.append(get_distance())
