from typing import Optional
import threading

# Modbus addresses and constants (assumed from modbus_test/scan.js)
POS_ADDR = 51            # Input registers address (2 regs -> 32-bit position)
POS_QTY = 2
MOVE_ABS_ADDR = 254      # Holding registers address for absolute move (4 regs)


class Servo42CProtocol:
    """Modbus RTU protocol implementation for Servo42D devices.

    We preserve the class name to minimize changes to import sites.
    """

    def __init__(self, device: str, baud_rate: int, logger=None, timeout: float = 1.0):
        self.device = f'/dev/{device}' if not device.startswith('/dev/') else device
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.logger = logger
        self.client: Optional[object] = None
        self._io_lock = threading.Lock()

    def connect(self) -> bool:
        try:
            if self.client is not None:
                try:
                    self.client.close()
                except Exception:
                    pass

            # Lazy import to allow simulation without pymodbus installed
            from pymodbus.client import ModbusSerialClient
            import time
            self.client = ModbusSerialClient(
                method='rtu',
                port=self.device,
                baudrate=self.baud_rate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=self.timeout,
                retries=2,
                retry_on_empty=True,
                strict=False,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
            ok = self.client.connect()
            # Give the RS485 adapter/device a brief settle time
            if ok:
                time.sleep(0.1)
            if ok and self.logger:
                self.logger.info(f'Connected to {self.device} at {self.baud_rate} baud (Modbus RTU)')
            if not ok and self.logger:
                self.logger.error('Failed to establish Modbus RTU connection')
            return bool(ok)
        except Exception as e:
            if self.logger:
                self.logger.error(f'Modbus connect error: {e}')
            return False

    def _ensure(self) -> None:
        if self.client is None:
            raise RuntimeError('Modbus client not initialized')

    def get_pulses(self, id: int) -> int:
        """Read current absolute position in pulses (32-bit signed)."""
        self._ensure()
        last_error = None
        for attempt in range(3):
            try:
                with self._io_lock:
                    rr = self.client.read_input_registers(address=POS_ADDR, count=POS_QTY, slave=id)
                if hasattr(rr, 'isError') and rr.isError():
                    last_error = rr
                    # brief backoff before retry
                    import time
                    time.sleep(0.1)
                    continue
                regs = getattr(rr, 'registers', None)
                if not regs or len(regs) != 2:
                    last_error = RuntimeError(f'Unexpected position register count: {len(regs) if regs is not None else "none"}')
                    import time
                    time.sleep(0.1)
                    continue
                value = ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)
                # Convert to signed 32-bit
                if value & 0x80000000:
                    value = -((~value & 0xFFFFFFFF) + 1)
                return int(value)
            except Exception as e:
                last_error = e
                import time
                time.sleep(0.1)
        raise RuntimeError(f'Modbus read position failed: {last_error}')

    def move_absolute(self, id: int, acceleration: int, speed: int, abs_pulses: int) -> bool:
        """Issue absolute move command via holding registers.

        Data layout: [accel, speed, pos_hi, pos_lo].
        """
        self._ensure()
        pos_hi = (abs_pulses >> 16) & 0xFFFF
        pos_lo = abs_pulses & 0xFFFF
        values = [acceleration & 0xFFFF, speed & 0xFFFF, pos_hi, pos_lo]
        with self._io_lock:
            wr = self.client.write_registers(address=MOVE_ABS_ADDR, values=values, slave=id)
        if wr.isError():
            if self.logger:
                self.logger.error(f'Modbus write move_absolute failed: {wr}')
            return False
        return True

    def stop(self, id: int) -> None:
        """Broadcast emergency stop using vendor function 0xF7 to unit 0."""
        self._ensure()
        try:
            from pymodbus.pdu import ModbusRequest

            class EmergencyStopRequest(ModbusRequest):
                function_code = 0xF7

                def __init__(self):
                    super().__init__()

                def encode(self) -> bytes:
                    # No payload for emergency stop
                    return b''

                def decode(self, data: bytes) -> None:
                    # No response expected for broadcast
                    return None

                def get_response_pdu_size(self) -> int:
                    # No response expected for broadcast requests
                    return 0

            req = EmergencyStopRequest()
            try:
                # Broadcast to all units; most devices won't send a response
                with self._io_lock:
                    self.client.execute(req, slave=0)
            except Exception:
                # Some client versions may not support execute() with custom requests cleanly.
                # Since this is a broadcast fire-and-forget, ignore errors here.
                pass
        except Exception as e:
            if self.logger:
                self.logger.error(f'Failed to send emergency stop: {e}')

    def close(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
                if self.logger:
                    self.logger.info('Modbus connection closed')
            finally:
                self.client = None
