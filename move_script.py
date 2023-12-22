import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

# З'єднання з дроном
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)


# Функція для зміни режиму AltHold
def set_alt_hold_mode():
    vehicle.mode = VehicleMode("ALT_HOLD")


# Функція для здійснення зліту
def takeoff(altitude):
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    vehicle.channels.overrides = {}
    vehicle.channels.overrides['3'] = 1700  # Підняти дрон
    time.sleep(2)

    # Поступове зменшення каналу 3 для збереження висоти
    for i in range(1700, 1500, -10):
        vehicle.channels.overrides['3'] = i
        time.sleep(0.1)

    vehicle.channels.overrides = {}


# Функція для визначення відстані між двома точками
def get_distance(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return math.sqrt(dlat ** 2 + dlong ** 2) * 1.113195e5  # Використовуємо формулу для переведення градусів у метри


# Функція для руху вперед на задану відстань
def move_forward(distance):
    initial_location = vehicle.location.global_relative_frame
    target_location = initial_location.get_location_ned(0, distance, 0)

    while get_distance(vehicle.location.global_relative_frame, target_location) > 1:
        vehicle.channels.overrides = {}
        vehicle.channels.overrides['1'] = 1500  # Рух вперед
        time.sleep(0.1)

    vehicle.channels.overrides = {}


# Функція для повороту на азимут (yaw)
def rotate_yaw(yaw):
    vehicle.channels.overrides = {}
    vehicle.channels.overrides['4'] = 1500 + int(yaw / 2)  # Поворот


# Змінні для точок A та B (нові значення)
point_a = (50.450739, 30.461242, 100)
point_b = (50.443326, 30.448078, 100)

# Зліт
set_alt_hold_mode()
takeoff(point_a[2])

# Визначення відстані між точками A та B
distance_ab = get_distance(LocationGlobalRelative(*point_a), LocationGlobalRelative(*point_b))

# Рух вперед на визначену відстань
move_forward(distance_ab)

# Поворот на азимут 350 градусів
rotate_yaw(350)

# Закриття з'єднання з дроном
vehicle.close()
