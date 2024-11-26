from machine import Pin, ADC, PWM, I2C
from lcd_api import LcdApi
from pico_i2c_lcd import I2cLcd
import time

# Pin Definitions
POWER_BTN = Pin(2, Pin.IN, Pin.PULL_UP)
ALERT_BTN = Pin(3, Pin.IN, Pin.PULL_UP)
RED_LED = Pin(4, Pin.OUT)
GREEN_LED = Pin(5, Pin.OUT)
BUZZER = PWM(Pin(6))
AIR_PUMP_RELAY = Pin(7, Pin.OUT)
SOLENOID_RELAY = Pin(8, Pin.OUT)
PRESSURE_SENSOR = ADC(Pin(A0))  # Analog input for pressure sensor
ADJUST_PRESSURE = ADC(Pin(A1))   # Analog input for pressure adjustment knob

# LCD Display Setup
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
lcd = I2cLcd(i2c, 0x27, 2, 16)

# Constants
MAX_PRESSURE = 10  # Maximum pressure in Bar
PRESSURE_TOLERANCE = 0.2  # Tolerance range for pressure matching
RELEASE_RATE = 1  # Bar per minute for controlled release
BUZZER_FREQ = 420  # ISO/IEC 60601-1-8 high-priority alarm frequency
BUZZER_PATTERN = [(0.1, 0.1), (0.1, 0.1), (0.1, 0.1)]  # ISO/IEC 60601-1-8 high-priority alarm pattern

class BalloonAngioplasty:
    def __init__(self):
        self.desired_pressure = 0
        self.current_pressure = 0
        self.system_active = False
        self.alert_mode = False
        
        # Initialize outputs
        self.RED_LED.off()
        self.GREEN_LED.off()
        self.AIR_PUMP_RELAY.off()
        self.SOLENOID_RELAY.off()
        BUZZER.duty_u16(0)  # Turn off buzzer
        
        # Clear LCD display
        lcd.clear()
        
    def read_pressure_sensor(self):
        # Convert ADC reading to pressure (Bar)
        # Assuming 0-40kPa sensor maps to 0-10 Bar
        adc_value = PRESSURE_SENSOR.read_u16()
        return (adc_value / 65535) * MAX_PRESSURE
    
    def read_desired_pressure(self):
        # Read from adjustable resistor
        adc_value = ADJUST_PRESSURE.read_u16()
        return (adc_value / 65535) * MAX_PRESSURE
    
    def activate_buzzer(self):
        # High-priority alarm with five rising tones
        tones = [800, 1000, 1200, 1400, 1600]  # Frequencies for rising tones
        tone_duration = 0.1  # Duration of each tone (100 ms)
        pause_duration = 0.1  # Silence between tones (100 ms)
        burst_pause = 0.5  # Pause between bursts (500 ms)
        
        for _ in range(2):  # Repeat burst twice (example; adjust as needed)
            for freq in tones:
                BUZZER.freq(freq)
                BUZZER.duty_u16(32768)  # 50% duty cycle
                time.sleep(tone_duration)  # Play tone
                BUZZER.duty_u16(0)
                time.sleep(pause_duration)  # Pause between tones
            time.sleep(burst_pause)  # Pause between bursts
        
    def deactivate_buzzer(self):
        BUZZER.duty_u16(0)
        
    def controlled_release(self):
        # Open solenoid valve for controlled release
        self.SOLENOID_RELAY.on()
        # Calculate release time based on current pressure and release rate
        release_time = (self.current_pressure / RELEASE_RATE) * 60
        time.sleep(release_time)
        self.SOLENOID_RELAY.off()
        
    def display_pressure(self):
        lcd.clear()
        lcd.move_to(0, 0)
        lcd.putstr(f"Desired: {self.desired_pressure:.2f} Bar")
        lcd.move_to(0, 1)
        lcd.putstr(f"Current: {self.current_pressure:.2f} Bar")
        
    def run(self):
        while True:
            if not POWER_BTN.value():  # Power button pressed (active low)
                self.system_active = True
                
            if self.system_active:
                self.desired_pressure = self.read_desired_pressure()
                self.current_pressure = self.read_pressure_sensor()
                
                # Check if pressure matches desired pressure
                pressure_diff = abs(self.current_pressure - self.desired_pressure)
                
                if pressure_diff <= PRESSURE_TOLERANCE:
                    # Pressure matched
                    self.RED_LED.off()
                    self.GREEN_LED.on()
                    self.AIR_PUMP_RELAY.off()
                    self.deactivate_buzzer()
                    
                    if not ALERT_BTN.value():  # Alert button pressed
                        self.controlled_release()
                        self.system_active = False  # End procedure
                        
                else:
                    # Pressure doesn't match
                    self.GREEN_LED.off()
                    self.RED_LED.on()
                    
                    if self.current_pressure < self.desired_pressure:
                        # Need to increase pressure
                        self.AIR_PUMP_RELAY.on()
                    else:
                        # Pressure too high
                        self.AIR_PUMP_RELAY.off()
                        self.activate_buzzer()
                        self.controlled_release()
                        
                # Display desired and current pressure on LCD
                self.display_pressure()
                        
            else:
                # System inactive
                self.RED_LED.off()
                self.GREEN_LED.off()
                self.AIR_PUMP_RELAY.off()
                self.SOLENOID_RELAY.off()
                self.deactivate_buzzer()
                lcd.clear()
                
            time.sleep(0.1)  # Small delay to prevent tight polling

# Create and run the system
if __name__ == "__main__":
    angioplasty = BalloonAngioplasty()
    angioplasty.run()
