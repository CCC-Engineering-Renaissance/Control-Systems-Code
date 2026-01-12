# Thruster.h design notes

## 'explicit Thruster(int pin)'
    This constructor creates a 'Thruster' object and binds it to a PCA9685 output pin

    - 'pin' refers to a PCA9685 output channel (0-15)
    - It is not associated with the Raspberry Pi GPIO pin

## Why 'explicit' is required
        - This makes it so you dont end up making a thruster object on accident from an integer
        - The only way to make a thruster and connect with one pin is the format Thruster thruster(int pin) 
          and you cannot compile if you write Thruster thruster = 'integer'

## 'Thruster(int pin, int rest)'
    This constructor creates a 'Thruster' object and binds it to a PCA9685 output pin AND decide what PWM value is to stop the thruster object or enter 'rest'
    
    - Usually we would like 1500 us as our PWM value to stop the Thrusters, but not every Thruster object will stop perfectly at that value.
    - This line gives us the option to adjust a rest value for a specific Object that isn't stopping at 1500 us
    - We could say Thruster thruster_3(3, 1496) if 1496 us is more responsive than 1500 us

    
    
    
