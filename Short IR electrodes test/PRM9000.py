import logging
import time
import pyvisa


class PRM9000:
    def __init__(self, time_offset, resource_name, cpm_to_uSv=0.002857):
        self.time_offset = time_offset
        self.cpm_to_uSv = cpm_to_uSv
        self.rm = pyvisa.ResourceManager()
        try:
            self.instr = self.rm.open_resource(resource_name)
        except pyvisa.errors.VisaIOError as err:
            self.verification_string = str(err)
            self.instr = False
            return

        # Configure serial parameters for PRM-9000
        self.instr.baud_rate = 57600
        self.instr.data_bits = 8
        self.instr.parity = pyvisa.constants.Parity.none
        self.instr.stop_bits = pyvisa.constants.StopBits.two  # 2 stop bits required
        self.instr.timeout = 1000  # ms

        try:
            self.verification_string = self.QueryIdentification()
        except pyvisa.errors.VisaIOError as err:
            self.verification_string = str(err)

        self.new_attributes = []
        self.dtype = "f"
        self.shape = (2,)
        self.warnings = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if self.instr:
            self.instr.close()

    def QueryIdentification(self):
        line = self._read_line()
        return f"first line: {line}" if line else "no data"

    def ReadValue(self):
        return [time.time() - self.time_offset, self.ReadDoseRate()]

    def ReadDoseRate(self):
        for _ in range(10):
            line = self._read_line()
            print(f"[PRM-9000] Raw line: {line}")
            parts = line.split(",")
            if len(parts) == 4 and parts[-1].strip().upper() == "CPM":
                try:
                    cpm = int(parts[2])
                    return cpm * self.cpm_to_uSv  # µSv/h
                except ValueError:
                    continue
        return -1

    def GetWarnings(self):
        warnings = self.warnings
        self.warnings = []
        return warnings

    def _read_line(self, method="read"):
        if method == "read":
            try:
                return self.instr.read().strip()
            except pyvisa.errors.VisaIOError:
                return ""
        else:
            # Byte-by-byte fallback
            line = b""
            while True:
                try:
                    byte = self.instr.read_bytes(1)
                    if byte == b"\n":
                        break
                    line += byte
                except pyvisa.errors.VisaIOError:
                    continue
            return line.decode(errors="ignore").strip()
