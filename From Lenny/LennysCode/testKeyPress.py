# https://sshkeyboard.readthedocs.io/en/latest/

from sshkeyboard import listen_keyboard

print("\nPress P to quit")

def press(key):
    print(f"'{key}' pressed")
        
def release(key):
    print(f"'{key}' released")

listen_keyboard(
    on_press=press,
    on_release=release,
    until="p",
)