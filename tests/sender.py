from machine import Pin, SPI
from time import sleep
from ulora.core import ULoRa  # Ensure the ULoRa class is implemented and imported correctly

# ============================================================================ 
# Sender Test Example
# ============================================================================ 
if __name__ == "__main__": 
    # This example is designed for a MicroPython environment with an SX127x connected. 
    # Adjust the SPI bus and pin numbers as per your hardware configuration.
    try: 
        # ------------------------- Initializing SPI -------------------------
        print("Initializing SPI bus...")
        spi = SPI(1, baudrate=5000000, polarity=0, phase=0,
                  sck=Pin(25), mosi=Pin(27), miso=Pin(26))
        print("SPI bus initialized with SCK: 25, MOSI: 27, MISO: 26.")
        
        # ------------------------- Defining Pin Mappings --------------------
        print("Setting up pin configurations...")
        pins = {
            "ss": 14,     # Chip Select (CS) pin
            "reset": 32,  # Reset pin
            "dio0": 33    # DIO0 pin
        }
        print(f"Pin configuration: SS={pins['ss']}, Reset={pins['reset']}, DIO0={pins['dio0']}.")
        
        # ------------------------- Creating ULoRa Instance ------------------
        print("Creating ULoRa instance with default parameters...")
        lora = ULoRa(spi, pins)
        print("ULoRa instance created successfully.")
        
        # ------------------------- Transmitting Test Message ----------------
        test_message = "Hello From Arman Ghobadi"
        print("\n----- Transmitting Message -----")
        print(f"Message: {test_message}")
        
        # Send the message via LoRa
        lora.println(test_message)
        
        print("Message transmission complete.")
        print("---------------------------------------------------------------------\n")
        
        # ------------------------- Waiting for Response ---------------------
        # You can add code here to listen for incoming messages if needed.
        print("You can now listen for responses...")

    except Exception as e:
        # ------------------------- Error Handling --------------------------
        print("\nError during test:")
        print(f"Exception: {e}")
        print("Please check the wiring and LoRa module configuration.")

