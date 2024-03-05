import inputs
import os

class Controller:
    def __init__(self):
        """Initialize the joystick components"""
        self.devices = inputs.devices.gamepads
        if not self.devices:
            # No gamepad detected
            print("No gamepad detected")
            exit()
        
        print("Gamepad detected")
        
    def listen(self):
        """Listen for events to happen"""
        axis_data = {}
        events = inputs.get_gamepad()
        for event in events:
            if event.ev_type == 'Absolute':
                axis_data[event.code] = event.state
        return axis_data

if __name__ == "__main__":
    ps5_controller = Controller()
    
    try:
        while True:
            os.system('cls' if os.name == 'nt' else 'clear') 
            axis_data = ps5_controller.listen()
            print(axis_data)
    except KeyboardInterrupt:
        print("\nExiting program.")
