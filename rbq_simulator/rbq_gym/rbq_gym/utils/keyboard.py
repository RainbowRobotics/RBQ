from pynput import keyboard

class KeyboardController:
    def __init__(self, command_ranges):
        self.velocity_commands = [0.0, 0.0, 0.0]
        self.base_control_commands = [0.0, 0.0, 0.0]
        self.key_states = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False, 'z': False, 'x': False, 'c': False, 'v': False, 'b': False, 'n': False, 'm': False}
        self.running = True
        self.command_ranges = command_ranges

    def on_press(self, key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            if key.char in self.key_states:
                self.key_states[key.char] = False

        except AttributeError:
            pass

        if key == keyboard.Key.esc:
            self.running = False
            return False

    def update_velocity(self):
        # lin_vel_x
        if self.key_states['w'] and self.key_states['s'] == False:
            self.velocity_commands[0] += 0.05
        if self.key_states['s'] and self.key_states['w'] == False:
            self.velocity_commands[0] -= 0.05
        
        # lin_vel_y
        if self.key_states['a'] and self.key_states['d'] == False:
            self.velocity_commands[1] += 0.05
        if self.key_states['d'] and self.key_states['a'] == False:
            self.velocity_commands[1] -= 0.05
        
        # ang_vel_yaw
        if self.key_states['q'] and self.key_states['e'] == False:
            self.velocity_commands[2] += 0.05
        if self.key_states['e'] and self.key_states['q'] == False:
            self.velocity_commands[2] -= 0.05 
                
    def update_commands(self, env):
        if self.key_states['w'] == False and self.key_states['s'] == False:
            self.velocity_commands[0] = self.velocity_commands[0] * 0.95
            
        if self.key_states['a'] == False and self.key_states['d'] == False:
            self.velocity_commands[1] = self.velocity_commands[1] * 0.95
            
        if self.key_states['q'] == False and self.key_states['e'] == False:
            self.velocity_commands[2] = self.velocity_commands[2] * 0.95

        max_vel_cmd = 2.0
        if self.key_states['m'] == True:
            max_vel_cmd *= 2.0
        if self.key_states['n'] == True:
            max_vel_cmd *= 1.5
                    
        if self.velocity_commands[0] >= 0:
            self.velocity_commands[0] = min(max_vel_cmd, self.velocity_commands[0])
        elif self.velocity_commands[0] < 0:
            self.velocity_commands[0] = max(-max_vel_cmd, self.velocity_commands[0])

        if self.velocity_commands[1] >= 0:
            self.velocity_commands[1] = min(self.command_ranges.lin_vel_y[1]+0.3, self.velocity_commands[1])
        elif self.velocity_commands[1] < 0:
            self.velocity_commands[1] = max(self.command_ranges.lin_vel_y[0], self.velocity_commands[1])

        if self.velocity_commands[2] >= 0:
            self.velocity_commands[2] = min(1.5, self.velocity_commands[2])
        elif self.velocity_commands[2] < 0:
            self.velocity_commands[2] = max(-1.5, self.velocity_commands[2])
            
        for j in range(3):
            env.commands[:, j] = self.velocity_commands[j]

    def start_listener(self):
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()