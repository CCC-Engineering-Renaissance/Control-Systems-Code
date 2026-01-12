# Thruster.h design notesi

Useful Statement: If a method does not promise motion, it must not require the driver.

### Constructors

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

    
## 'Thruster(int pin, int rest, int offset)'
    This constructor creates a 'Thruster' object and binds it to a PCA9685 output pin, decides what PWM value is to stop the thruster object, and define how far the thruster is allowed to deviate from rest.

    - It is the same concept as Thruster(int pin, int rest) where the rest value isn't always perfect for every thruster. The same can be said for the offset value.
    
### Set Methods
   
## 'void setPWM(int pwm_us, PiPCA9685::PCA9685& driver)'
    Sends exact PWM value to PCA9685 output pin that the thruster is bound to.

    - We use the driver class because the driver is what converses with the hardware to get the thrusters to move.
    
## 'void setPower(double power, PiPCA9685::PCA9685 &driver)'
    Exists to be able to produce a certain amount of thrust at this instant.

    - The intent is to be able to command thrusters with a normalized value instead of raw PWM values

## 'void stop(PiPCA9685::PCA9685 &driver);'
    Thruster will be able to enter a neatral state with this.

    - Same idea with setPower above, however this is an option to completely stop the thrusters 

## 'setRest(int rest_us)', 'setOffset(int offset_us)', and 'setLimits(int min_us, int max_us)'
    These last Set Methods share a similar goal.

    - They will be able to change internal rules that will be used for future commands
    - We may start with an object like Thruster thruster_3(3, 1500, 400) and later we have a situation where rather than changing our constructor
      values we would rather use these set methods.

### Get Methods


