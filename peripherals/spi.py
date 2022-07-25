#
# This file is part of Thunderscope-LiteX project.
#
# Copyright (c) 2021 Felix Domke <tmbinc@elitedvb.net>
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E S C R I P T I O N                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                               D E F I N I T I O N S                                              #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

SPI_CONTROL_START  = (1 << 0)
SPI_CONTROL_LENGTH = (1 << 8)
SPI_STATUS_DONE    = (1 << 0)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  G A T E W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

# N/A.

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #
#                                  S O F T W A R E                                                 #
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ #

class SPIDriver:
    def __init__(self, bus, name):
        self.control = getattr(bus.regs, f"{name}_control")
        self.status  = getattr(bus.regs, f"{name}_status")
        self.cs      = getattr(bus.regs, f"{name}_cs")
        self.mosi    = getattr(bus.regs, f"{name}_mosi")

    def write(self, cs, data):
        assert len(data) <= 3
        # Convert data to bytes (if not already).
        data = data if isinstance(data, (bytes, bytearray)) else bytes(data)
        # Set Chip Select.
        self.cs.write((1 << cs))
        # Prepare MOSI data.
        mosi_bits = len(data)*8
        mosi_data = int.from_bytes(data, byteorder="big")
        mosi_data <<= (24 - mosi_bits)
        self.mosi.write(mosi_data)
        # Start SPI Xfer.
        self.control.write(mosi_bits*SPI_CONTROL_LENGTH | SPI_CONTROL_START)
        # Wait SPI Xfer to be done.
        while not (self.status.read() & SPI_STATUS_DONE):
            pass
