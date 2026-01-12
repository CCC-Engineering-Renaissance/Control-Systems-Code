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
    
    
    
    
