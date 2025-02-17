"""
ulora.py

ULoRa: A lightweight library for the SX127x LoRa module.
This library provides functionalities for configuring and communicating with the SX127x.
It supports functions for setting frequency, TX power, bandwidth, spreading factor, etc.,
as well as packet transmission and reception.

"""

import gc
import machine
from machine import SPI, Pin
from utime import ticks_ms, sleep_ms  # ticks_ms and sleep_ms imported from utime
from micropython import const

# ============================================================================
# SX127x Register Definitions
# ============================================================================
REG_FIFO                = const(0x00)
REG_OP_MODE             = const(0x01)
REG_FRF_MSB             = const(0x06)
REG_FRF_MID             = const(0x07)
REG_FRF_LSB             = const(0x08)
REG_PA_CONFIG           = const(0x09)
REG_LNA                 = const(0x0C)
REG_FIFO_ADDR_PTR       = const(0x0D)
REG_FIFO_TX_BASE_ADDR   = const(0x0E)
REG_FIFO_RX_BASE_ADDR   = const(0x0F)
REG_FIFO_RX_CURRENT_ADDR= const(0x10)
REG_IRQ_FLAGS_MASK      = const(0x11)
REG_IRQ_FLAGS           = const(0x12)
REG_RX_NB_BYTES         = const(0x13)
REG_PKT_SNR_VALUE       = const(0x19)
REG_PKT_RSSI_VALUE      = const(0x1A)
REG_MODEM_CONFIG_1      = const(0x1D)
REG_MODEM_CONFIG_2      = const(0x1E)
REG_PREAMBLE_MSB        = const(0x20)
REG_PREAMBLE_LSB        = const(0x21)
REG_PAYLOAD_LENGTH      = const(0x22)
REG_FIFO_RX_BYTE_ADDR   = const(0x25)
REG_MODEM_CONFIG_3      = const(0x26)
REG_RSSI_WIDEBAND       = const(0x2C)
REG_DETECTION_OPTIMIZE  = const(0x31)
REG_DETECTION_THRESHOLD = const(0x37)
REG_SYNC_WORD           = const(0x39)
REG_DIO_MAPPING_1       = const(0x40)
REG_VERSION             = const(0x42)
REG_INVERTIQ            = const(0x33)
REG_INVERTIQ2           = const(0x3B)

# ============================================================================
# SX127x Mode and Power Settings
# ============================================================================
MODE_LONG_RANGE_MODE    = const(0x80)
MODE_SLEEP              = const(0x00)
MODE_STDBY              = const(0x01)
MODE_TX                 = const(0x03)
MODE_RX_CONTINUOUS      = const(0x05)
MODE_RX_SINGLE          = const(0x06)

PA_OUTPUT_RFO_PIN       = const(0)
PA_OUTPUT_PA_BOOST_PIN  = const(0x01)
PA_BOOST                = const(0x80)

# ============================================================================
# IRQ Masks
# ============================================================================
IRQ_TX_DONE_MASK        = const(0x08)
IRQ_PAYLOAD_CRC_ERROR_MASK = const(0x20)
IRQ_RX_DONE_MASK        = const(0x40)
IRQ_RX_TIME_OUT_MASK    = const(0x80)

# ============================================================================
# IQ Inversion Constants
# ============================================================================
RFLR_INVERTIQ_RX_MASK   = const(0xBF)
RFLR_INVERTIQ_RX_OFF    = const(0x00)
RFLR_INVERTIQ_RX_ON     = const(0x40)
RFLR_INVERTIQ_TX_MASK   = const(0xFE)
RFLR_INVERTIQ_TX_OFF    = const(0x01)
RFLR_INVERTIQ_TX_ON     = const(0x00)
RFLR_INVERTIQ2_ON       = const(0x19)
RFLR_INVERTIQ2_OFF      = const(0x1D)

# ============================================================================
# Other Definitions
# ============================================================================
MAX_PKT_LENGTH = const(255)
FifoTxBaseAddr = const(0x00)
FifoRxBaseAddr = const(0x00)

# ============================================================================
# Default LoRa Parameters
# ============================================================================
DEFAULT_PARAMETERS = {
    "frequency": 433000000,
    "frequency_offset": 0,
    "tx_power_level": 10,
    "signal_bandwidth": 125e3,
    "spreading_factor": 9,
    "coding_rate": 5,
    "preamble_length": 8,
    "implicitHeader": False,
    "sync_word": 0x12,
    "enable_CRC": True,
    "invert_IQ": False,
}

# ============================================================================
# ULoRa Class Definition
# ============================================================================
class ULoRa:
    """
    ULoRa class to interface with the SX127x LoRa module.
    """
    def __init__(self, spi, pins, parameters=None):
        """
        Initialize the LoRa module.
        
        :param spi: Initialized SPI object.
        :param pins: Dictionary with pin mappings, e.g.,
                     {"ss": <pin>, "reset": <pin>, "dio0": <pin>}.
        :param parameters: (Optional) Dictionary with LoRa configuration parameters.
        """
        self.spi = spi
        self.pins = pins
        self.parameters = DEFAULT_PARAMETERS.copy()
        if parameters:
            self.parameters.update(parameters)
        
        # Setup slave select (CS) pin
        self.pin_ss = Pin(self.pins["ss"], Pin.OUT)
        
        # Setup reset pin if provided and perform a hardware reset
        if "reset" in self.pins:
            self.pin_reset = Pin(self.pins["reset"], Pin.OUT)
            self.reset_module()
        else:
            self.pin_reset = None
        
        self.lock = False
        self.implicit_header_mode = None
        
        # Check LoRa module version
        version = None
        for _ in range(5):
            version = self.read_register(REG_VERSION)
            if version:
                break
        print("SX127x Version: {}".format(version))
        if version != 0x12:  # Expected version is 0x12 (18 in decimal)
            print("Bad LoRa Connection! Version: {}".format(version))
            machine.reset()
        else:
            print("LoRa Connection OK! Version: {}".format(version))
        
        # Put module in sleep mode for configuration
        self.sleep()
        
        # Configure LoRa parameters
        self.set_frequency(self.parameters["frequency"])
        self.set_signal_bandwidth(self.parameters["signal_bandwidth"])
        
        # Enable LNA boost and auto AGC
        self.write_register(REG_LNA, self.read_register(REG_LNA) | 0x03)
        self.write_register(REG_MODEM_CONFIG_3, 0x04)
        
        self.set_tx_power(self.parameters["tx_power_level"])
        self.set_implicit_header(self.parameters["implicitHeader"])
        self.set_spreading_factor(self.parameters["spreading_factor"])
        self.set_coding_rate(self.parameters["coding_rate"])
        self.set_preamble_length(self.parameters["preamble_length"])
        self.set_sync_word(self.parameters["sync_word"])
        self.enable_crc(self.parameters["enable_CRC"])
        self.invert_iq(self.parameters["invert_IQ"])
        
        # Enable LowDataRateOptimize if symbol duration > 16ms
        bw = self.parameters["signal_bandwidth"]
        sf = self.parameters["spreading_factor"]
        if (1000 / bw / (2 ** sf)) > 16:
            self.write_register(REG_MODEM_CONFIG_3, self.read_register(REG_MODEM_CONFIG_3) | 0x08)
        
        # Set FIFO base addresses
        self.write_register(REG_FIFO_TX_BASE_ADDR, FifoTxBaseAddr)
        self.write_register(REG_FIFO_RX_BASE_ADDR, FifoRxBaseAddr)
        
        self.standby()
    
    def reset_module(self):
        """
        Perform a hardware reset of the LoRa module using the reset pin.
        """
        if self.pin_reset is None:
            return
        # Drive reset pin low for 100ms then high
        self.pin_reset.value(0)
        sleep_ms(100)
        self.pin_reset.value(1)
        sleep_ms(100)
    
    # ---------------------------
    # Packet Transmission Methods
    # ---------------------------
    def begin_packet(self, implicit_header=None):
        """
        Prepare the module for packet transmission.
        
        :param implicit_header: (Optional) Boolean to override current header mode.
        """
        self.standby()
        if implicit_header is not None:
            self.set_implicit_header(implicit_header)
        # Reset FIFO pointer and payload length
        self.write_register(REG_FIFO_ADDR_PTR, FifoTxBaseAddr)
        self.write_register(REG_PAYLOAD_LENGTH, 0)
    
    def write(self, buffer):
        """
        Write data to the LoRa FIFO.
        
        :param buffer: Data bytes (or bytearray) to be sent.
        :return: Number of bytes written.
        """
        current_length = self.read_register(REG_PAYLOAD_LENGTH)
        size = len(buffer)
        # Ensure packet does not exceed maximum allowed length
        size = min(size, MAX_PKT_LENGTH - FifoTxBaseAddr - current_length)
        for byte in buffer:
            self.write_register(REG_FIFO, byte)
        # Update payload length
        self.write_register(REG_PAYLOAD_LENGTH, current_length + size)
        return size
    
    def end_packet(self):
        """
        Transmit the packet and wait until transmission is complete.
        """
        # Set module to TX mode
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX)
        # Wait until TX_DONE flag is set
        while (self.read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
            pass
        # Clear TX_DONE flag
        self.write_register(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)
    
    def println(self, message, implicit_header=None, repeat=1):
        """
        Transmit a text message.
        
        :param message: String message to transmit.
        :param implicit_header: (Optional) Boolean to override header mode.
        :param repeat: Number of times to send the message.
        """
        if isinstance(message, str):
            message = message.encode()
        self.begin_packet(implicit_header)
        self.write(message)
        for _ in range(repeat):
            self.end_packet()
        self.collect_garbage()
    
    # ---------------------------
    # Packet Reception Methods
    # ---------------------------
    def receive(self, size=0):
        """
        Set the module to continuous receive mode.
        
        :param size: Expected payload size. If >0, uses implicit header mode.
        """
        self.set_implicit_header(size > 0)
        if size > 0:
            self.write_register(REG_PAYLOAD_LENGTH, size & 0xFF)
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)
    
    def listen(self, timeout=1000):
        """
        Listen for an incoming packet for a specified timeout.
        
        :param timeout: Timeout in milliseconds.
        :return: Received payload as bytes, or None if timeout occurs.
        """
        self.receive()
        start = ticks_ms()
        while True:
            if self.received_packet():
                return self.read_payload()
            if ticks_ms() - start > timeout:
                return None
    
    def received_packet(self, size=0):
        """
        Check if a packet has been received.
        
        :param size: Expected payload size. If >0, uses implicit header mode.
        :return: True if a packet is received, otherwise False.
        """
        irq_flags = self.get_irq_flags()
        self.set_implicit_header(size > 0)
        if size > 0:
            self.write_register(REG_PAYLOAD_LENGTH, size & 0xFF)
        if irq_flags == IRQ_RX_DONE_MASK:
            return True
        else:
            # If not in single RX mode, reset FIFO pointer and enter single RX mode
            if self.read_register(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE):
                self.write_register(REG_FIFO_ADDR_PTR, FifoRxBaseAddr)
                self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)
        return False
    
    def read_payload(self):
        """
        Read the received payload from the FIFO.
        
        :return: Payload data as bytes.
        """
        # Set FIFO pointer to the current RX address
        self.write_register(REG_FIFO_ADDR_PTR, self.read_register(REG_FIFO_RX_CURRENT_ADDR))
        # Determine payload length based on header mode
        if self.implicit_header_mode:
            packet_length = self.read_register(REG_PAYLOAD_LENGTH)
        else:
            packet_length = self.read_register(REG_RX_NB_BYTES)
        payload = bytearray()
        for _ in range(packet_length):
            payload.append(self.read_register(REG_FIFO))
        self.collect_garbage()
        return bytes(payload)
    
    def get_irq_flags(self):
        """
        Retrieve and clear the IRQ flags.
        
        :return: IRQ flags value.
        """
        irq_flags = self.read_register(REG_IRQ_FLAGS)
        self.write_register(REG_IRQ_FLAGS, irq_flags)
        return irq_flags
    
    def packet_rssi(self, high_frequency=True):
        """
        Get the RSSI value of the last received packet.
        
        :param high_frequency: Boolean flag; if True, uses high frequency offset.
        :return: Adjusted RSSI value.
        """
        rssi = self.read_register(REG_PKT_RSSI_VALUE)
        return rssi - (157 if high_frequency else 164)
    
    def packet_snr(self):
        """
        Get the SNR (Signal-to-Noise Ratio) of the last packet.
        
        :return: SNR value.
        """
        return self.read_register(REG_PKT_SNR_VALUE) * 0.25
    
    # ---------------------------
    # Module Mode Methods
    # ---------------------------
    def standby(self):
        """
        Set the module to standby mode.
        """
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY)
    
    def sleep(self):
        """
        Put the module into sleep mode.
        """
        self.write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)
    
    # ---------------------------
    # Configuration Methods
    # ---------------------------
    def set_tx_power(self, level, output_pin=PA_OUTPUT_PA_BOOST_PIN):
        """
        Set the transmission power level.
        
        :param level: Power level.
        :param output_pin: PA output type (PA_OUTPUT_RFO_PIN or PA_OUTPUT_PA_BOOST_PIN).
        """
        self.parameters["tx_power_level"] = level
        if output_pin == PA_OUTPUT_RFO_PIN:
            level = min(max(level, 0), 14)
            self.write_register(REG_PA_CONFIG, 0x70 | level)
        else:
            level = min(max(level, 2), 17)
            self.write_register(REG_PA_CONFIG, PA_BOOST | (level - 2))
    
    def set_frequency(self, frequency):
        """
        Set the operating frequency.
        
        :param frequency: Frequency in Hz.
        """
        self.parameters["frequency"] = frequency
        frequency += self.parameters["frequency_offset"]
        frf = (frequency << 19) // 32000000
        self.write_register(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self.write_register(REG_FRF_MID, (frf >> 8) & 0xFF)
        self.write_register(REG_FRF_LSB, frf & 0xFF)
    
    def set_spreading_factor(self, sf):
        """
        Set the spreading factor (6 to 12).
        
        :param sf: Spreading factor.
        """
        sf = min(max(sf, 6), 12)
        if sf == 6:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xC5)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0C)
        else:
            self.write_register(REG_DETECTION_OPTIMIZE, 0xC3)
            self.write_register(REG_DETECTION_THRESHOLD, 0x0A)
        current = self.read_register(REG_MODEM_CONFIG_2) & 0x0F
        self.write_register(REG_MODEM_CONFIG_2, current | ((sf << 4) & 0xF0))
    
    def set_signal_bandwidth(self, sbw):
        """
        Set the signal bandwidth.
        
        :param sbw: Bandwidth in Hz.
        """
        # Predefined bandwidth bins
        bins = (7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000)
        bw_index = 7  # Default to 125 kHz
        if sbw < 10:
            bw_index = int(sbw)
        else:
            for i, bw in enumerate(bins):
                if sbw <= bw:
                    bw_index = i
                    break
        current = self.read_register(REG_MODEM_CONFIG_1) & 0x0F
        self.write_register(REG_MODEM_CONFIG_1, current | (bw_index << 4))
    
    def set_coding_rate(self, denominator):
        """
        Set the coding rate.
        
        :param denominator: Denominator (between 5 and 8).
        """
        denominator = min(max(denominator, 5), 8)
        cr = denominator - 4
        current = self.read_register(REG_MODEM_CONFIG_1) & 0xF1
        self.write_register(REG_MODEM_CONFIG_1, current | (cr << 1))
    
    def set_preamble_length(self, length):
        """
        Set the preamble length.
        
        :param length: Preamble length.
        """
        self.write_register(REG_PREAMBLE_MSB, (length >> 8) & 0xFF)
        self.write_register(REG_PREAMBLE_LSB, length & 0xFF)
    
    def enable_crc(self, enable_crc):
        """
        Enable or disable CRC checking.
        
        :param enable_crc: Boolean flag.
        """
        modem_config_2 = self.read_register(REG_MODEM_CONFIG_2)
        if enable_crc:
            config = modem_config_2 | 0x04
        else:
            config = modem_config_2 & 0xFB
        self.write_register(REG_MODEM_CONFIG_2, config)
    
    def invert_iq(self, invert):
        """
        Invert the IQ signals.
        
        :param invert: Boolean flag.
        """
        self.parameters["invert_IQ"] = invert
        current = self.read_register(REG_INVERTIQ)
        if invert:
            new_val = (current & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_ON
            self.write_register(REG_INVERTIQ, new_val)
            self.write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_ON)
        else:
            new_val = (current & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF
            self.write_register(REG_INVERTIQ, new_val)
            self.write_register(REG_INVERTIQ2, RFLR_INVERTIQ2_OFF)
    
    def set_sync_word(self, sw):
        """
        Set the synchronization word.
        
        :param sw: Sync word.
        """
        self.write_register(REG_SYNC_WORD, sw)
    
    def set_implicit_header(self, implicit):
        """
        Set the module to use implicit or explicit header mode.
        
        :param implicit: Boolean flag.
        """
        if self.implicit_header_mode != implicit:
            self.implicit_header_mode = implicit
            modem_config_1 = self.read_register(REG_MODEM_CONFIG_1)
            if implicit:
                config = modem_config_1 | 0x01
            else:
                config = modem_config_1 & 0xFE
            self.write_register(REG_MODEM_CONFIG_1, config)
    
    # ---------------------------
    # Low-Level SPI Methods
    # ---------------------------
    def read_register(self, address):
        """
        Read a byte from the specified register.
        
        :param address: Register address.
        :return: Value read.
        """
        response = self.transfer(address & 0x7F)
        return int.from_bytes(response, 'big')
    
    def write_register(self, address, value):
        """
        Write a byte to the specified register.
        
        :param address: Register address.
        :param value: Value to write.
        """
        self.transfer(address | 0x80, value)
    
    def transfer(self, address, value=0x00):
        """
        Perform an SPI transfer.
        
        :param address: Register address.
        :param value: Byte to write.
        :return: Response as a bytearray.
        """
        response = bytearray(1)
        self.pin_ss.value(0)
        # Write the register address
        self.spi.write(bytes([address]))
        # Write the value and simultaneously read the response
        self.spi.write_readinto(bytes([value]), response)
        self.pin_ss.value(1)
        return response
    
    def dump_registers(self):
        """
        Dump the first 128 registers for debugging purposes.
        """
        for i in range(128):
            print("0x{:02X}: {:02X}".format(i, self.read_register(i)), end="")
            if (i+1) % 4 == 0:
                print()
            else:
                print(" | ", end="")
    
    def collect_garbage(self):
        """
        Run garbage collection.
        """
        gc.collect()



