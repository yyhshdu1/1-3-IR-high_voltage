import logging
import time

import numpy as np
import pyvisa

class RadEyeG20:
    def __init__(self, time_offset, resource_name):
        self.time_offset = time_offset
        self.rm = pyvisa.ResourceManager()
        try:
            self.instr = self.rm.open_resource(resource_name)
        except pyvisa.errors.VisaIOError as err:
            self.verification_string = str(err)
            self.instr = False
            return
        self.instr.baud_rate = 9600
        self.instr.data_bits = 7
        self.instr.parity = pyvisa.constants.Parity.even
        self.instr.stop_bits = pyvisa.constants.StopBits.two
        self.instr.timeout = 1000

        

        # make the verification string
        try:
            self.setup()
            self.verification_string = self.QueryIdentification()
        except pyvisa.errors.VisaIOError as err:
            try:
                self.verification_string = self.QueryIdentification()
            except pyvisa.errors.VisaIOError as err:
                self.verification_string = str(err)

        # HDF attributes generated when constructor is run
        self.new_attributes = []

        # shape and type of the array of returned data
        self.dtype = "f"
        self.shape = (2,)

        self.warnings = []

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        if self.instr:
            self.instr.close()

    def setup(self):
        self.instr.write_raw("@")
        self.instr.read_bytes(1)
        time.sleep(0.1)
        # auto send mode enabled
        self.instr.write("X1")
        self.instr.read()

    def ReadValue(self):
        return [time.time() - self.time_offset, self.ReadDoseRate()]

    def GetWarnings(self):
        warnings = self.warnings
        self.warnings = []
        return warnings

    #################################################################
    ##########           SERIAL COMMANDS                   ##########
    #################################################################

    def QueryIdentification(self):
        try:
            self.instr.clear()
            if 'FH41B2' in self.instr.read():
                return "connected"
            else:
                return "disconnected"
        except pyvisa.errors.VisaIOError as err:
            logging.warning("RadEyeG20 warning in QueryIdentification(): " + str(err))
            return str(err)

    def ReadDoseRate(self):
        resp = self.instr.read()
        dose_rate_nSv_h = 10*int(resp[2:].split(' ')[0])
        return dose_rate_nSv_h
