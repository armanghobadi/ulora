from machine import Pin, SPI
from time import sleep
from ulora.core import ULoRa  # Ensure the ULoRa class is implemented and imported correctly

# ============================================================================ 
# Receiver Test Example
# ============================================================================ 
if __name__ == "__main__": 
    # This example is designed for a MicroPython environment with an SX127x connected. 
    # Adjust the SPI bus and pin numbers as per your hardware configuration.
    try:
        # ------------------------- Initializing SPI -------------------------
        print("Initializing SPI bus...")
        # Initialize SPI with specified SCK, MOSI, MISO pins.
        spi = SPI(1, baudrate=5000000, polarity=0, phase=0,
                  sck=Pin(25), mosi=Pin(27), miso=Pin(26))
        print("SPI bus initialized with SCK: 25, MOSI: 27, MISO: 26.")
        
        # ------------------------- Defining Pin Mappings --------------------
        print("Setting up pin configurations...")
        # Define pin mappings based on your configuration:
        pins = {
            "ss": 14,     # Chip Select (CS) pin
            "reset": 32,  # Reset pin
            "dio0": 33    # DIO0 pin
        }
        print(f"Pin configuration: SS={pins['ss']}, Reset={pins['reset']}, DIO0={pins['dio0']}.")

        # ------------------------- Creating ULoRa Instance ------------------
        print("Creating ULoRa instance with default parameters...")
        # Create a ULoRa instance with default parameters
        lora = ULoRa(spi, pins)
        print("ULoRa instance created successfully.")
        
        # ------------------------- Listening for Incoming Message ----------- 
        print("\n----- Listening for Incoming Message -----")
        # Listen for an incoming message with a 20-second timeout
        payload = lora.listen(timeout=20000)  # 20000 ms = 20 seconds
        
        # ------------------------- Handling Received Message -----------------
        if payload:
            # If a message is received, print it
            print(f"Received payload: {payload}")
        else:
            # If no message is received within the timeout period
            print("No payload received within the timeout period.")
    
    except Exception as e:
        # ------------------------- Error Handling --------------------------
        print("Error during test:", e)
        print("Please check the wiring and LoRa module configuration.")

