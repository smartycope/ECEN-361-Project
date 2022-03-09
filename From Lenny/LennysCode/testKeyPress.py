# https://sshkeyboard.readthedocs.io/en/latest/

from sshkeyboard import listen_keyboard
import motor

print("\nPress P to quit")

def press(key):
    print(f"'{key}' pressed")
    if key == 'w':
        motor.directionTest(0)
    if key == 's':
        motor.directionTest(1)
    if key == 'a':
        motor.directionTest(2)
    if key == 'd':
        motor.directionTest(3)
        
def release(key):
    print(f"'{key}' released")

listen_keyboard(
    on_press=press,
    on_release=release,
    until="p",
)
