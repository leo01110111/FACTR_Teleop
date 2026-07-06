import evdev

paths = evdev.list_devices()

"""
PCsensor FootSwitch /dev/input/event25
PCsensor FootSwitch Mouse /dev/input/event24
PCsensor FootSwitch Keyboard /dev/input/event23
"""

main = evdev.InputDevice("/dev/input/event25")
mouse = evdev.InputDevice("/dev/input/event24")
keyboard = evdev.InputDevice("/dev/input/event23")

for event in keyboard.read_loop():
    print("value ", event.value)
    print("code ", event.code)

#/dev/input/event23
#left: code 30
#right: code 48
