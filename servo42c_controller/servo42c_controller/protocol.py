from typing import Optional
import threading
import time

# Modbus addresses and constants (assumed from modbus_test/scan.js)
POS_ADDR = 51            # Input registers address (2 regs -> 32-bit position)
POS_QTY = 2
MOVE_ABS_ADDR = 0xfe      # Holding registers address for absolute move (4 regs)


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
        start_overall = time.monotonic()
        if self.logger:
            self.logger.debug(f'Modbus get_pulses start id={id}')
        for attempt in range(3):
            try:
                with self._io_lock:
                    rr = self.client.read_input_registers(address=POS_ADDR, count=POS_QTY, slave=id)
                if hasattr(rr, 'isError') and rr.isError():
                    last_error = rr
                    # brief backoff before retry
                    time.sleep(0.1)
                    continue
                regs = getattr(rr, 'registers', None)
                if not regs or len(regs) != 2:
                    last_error = RuntimeError(f'Unexpected position register count: {len(regs) if regs is not None else "none"}')
                    time.sleep(0.1)
                    continue
                value = ((regs[0] & 0xFFFF) << 16) | (regs[1] & 0xFFFF)
                # Convert to signed 32-bit
                if value & 0x80000000:
                    value = -((~value & 0xFFFFFFFF) + 1)
                if self.logger:
                    self.logger.debug(f'Modbus get_pulses ok id={id} dt={(time.monotonic() - start_overall):.3f}s')
                return int(value)
            except Exception as e:
                last_error = e
                time.sleep(0.1)
        if self.logger:
            self.logger.error(f'Modbus get_pulses failed id={id} dt={(time.monotonic() - start_overall):.3f}s: {last_error}')
        raise RuntimeError(f'Modbus read position failed: {last_error}')

    def get_motor_status(self, id: int) -> int:
        """Read motor status via holding registers.
        """
        self._ensure()
        with self._io_lock:
            rd = self.client.read_input_registers(address=0xf1, count=1, slave=id)
        if rd.isError():
            if self.logger:
                self.logger.error(f'Modbus get_motor_status failed id={id} {rd}')
            return False
        regs = getattr(rd, 'registers', None)
        if self.logger:
            self.logger.info(f'Modbus get_motor_status ok value={regs} id={id}')
        return regs[0]

    def move_absolute(self, id: int, acceleration: int, speed: int, abs_pulses: int) -> bool:
        """Issue absolute move command via holding registers.

        Data layout: [accel, speed, pos_hi, pos_lo].
        """
        self._ensure()
        pos_hi = (abs_pulses >> 16) & 0xFFFF
        pos_lo = abs_pulses & 0xFFFF
        values = [acceleration & 0xFFFF, speed & 0xFFFF, pos_hi, pos_lo]
        start = time.monotonic()

        motor_status = self.get_motor_status(id)
        if motor_status > 1:
            if self.logger:
                self.logger.info(f'STATUS > 1, stopping id={id} status={motor_status}')
            # stop the motor
            wr = self.client.write_registers(address=MOVE_ABS_ADDR, values=[acceleration & 0xFFFF, 0, 0, 0], slave=id)
            for i in range(10):
                motor_status = self.get_motor_status(id)
                if motor_status <= 1:
                    if self.logger:
                        self.logger.info(f'STOPPED id={id} status={motor_status}')
                    break
                time.sleep(0.1)
            
        if self.logger:
            self.logger.info(f'Modbus move_absolute start id={id} accel={acceleration & 0xFFFF} speed={speed & 0xFFFF} pulses={abs_pulses}')
        with self._io_lock:
            wr = self.client.write_registers(address=MOVE_ABS_ADDR, values=values, slave=id)
        if wr.isError():
            if self.logger:
                self.logger.error(f'Modbus move_absolute failed id={id} dt={(time.monotonic() - start):.3f}s: {wr}')
            return False
        if self.logger:
            self.logger.info(f'Modbus move_absolute ok id={id} dt={(time.monotonic() - start):.3f}s')
        return True

    def stop(self, id: int, acceleration: int = 0xff) -> None:
        self._ensure()
        try:
            self.move_absolute(id=id, acceleration=acceleration, speed=0, abs_pulses=0)

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
