from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# З'єднання з дроном
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)

# Функція для зміни режиму AltHold
def set_alt_hold_mode():
    vehicle.mode = VehicleMode("ALT_HOLD")

# Функція для здійснення зліту
def takeoff(altitude):
    vehicle.simple_takeoff(altitude)

    # Чекаємо, поки дрон досягне висоти
    while True:
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Дрон досяг висоти")
            break
        time.sleep(1)

# Функція для переміщення до географічної точки
def goto_location(latitude, longitude, altitude):
    target_location = LocationGlobalRelative(latitude, longitude, altitude)
    vehicle.simple_goto(target_location)

    # Чекаємо, поки дрон досягне точки
    while True:
        distance = get_distance(vehicle.location.global_relative_frame, target_location)
        if distance < 1:
            print("Дрон досяг точки")
            break
        time.sleep(1)

# Функція для обчислення відстані між двома точками
def get_distance(location1, location2):
    dlat = location2.lat - location1.lat
    dlong = location2.lon - location1.lon
    return (dlat**2 + dlong**2)**0.5 * 1.113195e5

# Функція для повернення на заданий азимут
def set_heading(heading):
    vehicle.simple_goto(LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt,
        heading))

# Змінні для точок A та B (приклад значень)
point_a = (50.450739, 30.461242, 0)
point_b = (50.443326, 30.448078, 100)

# Зліт
takeoff(point_a[2])

# Зміна режиму на AltHold
set_alt_hold_mode()

# Переміщення до точки B
goto_location(point_b[0], point_b[1], point_b[2])

# Повернення на азимут 350 градусів
set_heading(350)

# Закриття з'єднання з дроном
vehicle.close()
