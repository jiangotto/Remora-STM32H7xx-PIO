LinuxCNC components

1. Add '-lgpiod' into /usr/share/linuxcnc/Makefile.modinc

2. Open this folder in a terminal, and install the components with command
sudo halcompile --install remora-spi.c

3. sudo chmod 666 /dev/gpiochip*

4. sudo chmod 666 /dev/spidev*