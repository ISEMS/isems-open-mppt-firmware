# isems-open-mppt-firmware

This repository contains the up-to-date firmware source and precompiled binary for the AVR ATmega8 microcontroller that is required for ISEMS. You can use the binary file main.hex and upload it to the Freifunk-OpenMPPT via simple serial port transfer.

If you have aquired a Freifunk-OpenMPPT recently, it is shipped with this firmware. So you can skip the following steps, unless you want to update.

More instructions and details about fuse bits and so on are available here: 

[Freifunk-OpenMPPT repository at Github](https://github.com/elektra42/freifunk-open-mppt) 

# How to flash/update the Freifunk-Open-MPP-Tracker

Whenever you power up the Freifunk-Open-MPP-Tracker, it will wait 8 seconds for the upload of a new firmware via the serial port.  So you can flash the Freifunk-Open-MPPT without a dedicated programmer :)

All you need is a 3.3 Volt level compatible serial port. Serial ports are now a legacy on Laptops and PCs, but there are cheap USB-serial-adapters available.  You will also need a serial port level shifter that is compatible with 3.3 Volt levels. You can get both from your hacker-friendly electronics store. There are also USB-to-RS232 adapters for 3.3 and 5 Volt logic levels available, since the Maker-community needs them for single board computers. You can also use the serial port of single board computer like RaspberryPi et al to flash your device.

Don't connect the Freifunk-Open-MPPT directly to a standard PC serial port or USB-to-serial dongle without a level shifter in between, as standard RS-232 ports use way higher signal voltages than 3.3 Volt. A standard-level serial port will certainly destroy the Atmega8!

If you are in the middle of nowhere and you can't get your hands on a serial port and/or level shifter, take a look at the bottom of this text.  There is a solution ;)


### Step one:


Power off the MPPT: Disconnect it from solar panel and battery.

The layout of the Freifunk-Open-MPPT interface header looks like this:

	MOSI Pin_1 * | * Pin_2  MISO
	SCLK Pin_3 * | * Pin_4  3.3V
	  TX Pin_5 * | * Pin_6  GND
	  RX Pin_7 * | * Pin_8  GND
	 RST Pin_9 * | * Pin_10 GND


Pin #1 is the pin next to the crystal. You have to make 4 connections. You need four jumper cables.

	3.3V -- 3.3V
	  TX -- RX
	  RX -- TX
	 GND -- GND

### Step two:


Actually, we can flash the firmware with just the *cat* command that every Unix-like operating systems has.  However, we need to configure the serial port of our PC/Laptop with the following settings beforehand:

	9600 baud
	8-N-1
	Hardware and software flow control (Xon/Xoff) disabled

I use the program 'minicom' to set the serial parameters of the serial port (Linux) via the 'minicom' menu.  Then I stop minicom without resetting the serial port (minicom will ask if it should do that before it stops, just choose 'no')

You can also use **putty** or any other terminal software, or **setserial** or **stty** to set up the serial port and **cat** to upload the firmware file.

### Step three:


Go to the directory with the new firmware. By default, the firmware is the file **main.hex**.  If you want to flash via a terminal software, select **main.hex** for upload.  The transfer protocol is ASCII. 

As an alternative, set up the serial port with **setserial** or **stty**, then type the command 

	cat main.hex > /dev/ttyUSB0

on the command line, if **ttyUSB** is your serial port. Don't press 'Return' yet.

If you use a different port, change /dev/tty-something accordingly.

### Step four:


Power on the Freifunk-Open-MPPT by connecting it to the battery. Start the file-upload within 8 seconds after connecting the power.

Wait until the upload is finished. Let another few seconds pass. Power-cycle the Freifunk-Open-MPPT again.

If the communication with the Open-MPPT doesn't work, you might have confused TX and RX lines. Swap the lines and try again. Other possible showstoppers to look for: Did you disable hardware and software flow control? Are the communication parameters correct? 


# I have no serial port and/or no level shifter

You can use the 3.3 V serial port of a WiFi router, running OpenWRT or LEDE.  In order to use the serial port of the router for flashing, you have to disable the serial root console.  To disable the serial root console of your router, comment out the line

	::askconsole:/bin/ash --login

in /etc/inittab

As long as you have disabled the serial console, don't accidently misconfigure your router, as you can no longer log into the device via serial port for debugging ;)

**minicom** or **picocom** are available as a software packages for **OpenWRT**.

You don't have to install a terminal software, if you know how to set up the serial port with the **setserial** or **stty** command line utility. **setserial** should be available as a software package. **stty** can be compiled into **busybox**, but your default router firmware might not have the app build in **busybox**.

The combination of **setserial** or **stty** and **cat** will save precious space in the tiny flash of your router ;)