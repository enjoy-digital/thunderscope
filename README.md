```
                ________                __
               /_  __/ /  __ _____  ___/ /__ _______ _______  ___  ___
                / / / _ \/ // / _ \/ _  / -_) __(_-</ __/ _ \/ _ \/ -_)
               /_/ /_//_/\_,_/_//_/\_,_/\__/_/ /___/\__/\___/ .__/\__/
                                                           /_/
                         FPGA gateware for Thunderscope hardware.
                                Powered by Migen & LiteX
```

![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)


LiteX based FPGA gateware for Thunderscope.
===========================================

This repo aims to provide a LiteX based gatwae for Thunderscope hardware.

<p align="center"><img src="https://user-images.githubusercontent.com/1450143/179495534-4c54973b-9203-4893-9eaa-d9177413e9bf.png" width="800"></p>

This repo is for now a WIP.

[> Prerequisites
----------------
- Python3, Vivado WebPACK
- Either a Vivado-compatible JTAG cable (native or XVCD), or OpenOCD.

[> Installing LiteX
-------------------
```sh
$ wget https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py
$ chmod +x litex_setup.py
$ sudo ./litex_setup.py init install
```

[> Build and Load the bitstream
--------------------------------
```sh
$ ./thunderscope --build --load
```

[> Open LiteX server
--------------------
```sh
$ litex_server.py --jtag --jtag-config=openocd_xc7_ft232.cfg
```
