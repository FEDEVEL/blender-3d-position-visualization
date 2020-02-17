====== OpenRex 3D Board Position Visualisation in Blender ======

I created a video about how to do it.
https://youtu.be/3Tjm3lMp4Co

----
====== APPENDIX ======
Down here you can find useful notes and codes.

===== Disable console output =====

We will be using serial port to communicate between OpenRex and Blender.

  - Boot up OpenRex to Linux and check IP address (run "ifconfig" command)<sxh bash>root@ubuntu-imx6:~# ifconfig
eth0      Link encap:Ethernet  HWaddr 00:0d:01:50:0d:ce
          inet addr:10.0.0.44  Bcast:10.0.0.255  Mask:255.255.255.0
          inet6 addr: 2601:206:4001:5f6f:20d:1ff:fe50:dce/64 Scope:Global
          inet6 addr: fe80::20d:1ff:fe50:dce/64 Scope:Link
          UP BROADCAST RUNNING MULTICAST  MTU:1500  Metric:1
          RX packets:15 errors:0 dropped:0 overruns:0 frame:0
          TX packets:17 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:1000
          RX bytes:2621 (2.6 KB)  TX bytes:2830 (2.8 KB)</sxh>
  - Use "reboot" command to go into uBoot
  - Once you are in uBoot, remove console information from Kernel command line. Depends what you are using, SD card or SATA drive, try "printenv" and look for "mmcargs" or "sataargs". It may look like this: <code>mmcargs=setenv bootargs console=${console},${baudrate} root=${mmcroot} ${video_args} ip=dhcp
sataargs=setenv bootargs console=${console},${baudrate} root=${sataroot} ${video_args} ip=dhcp</code>
  - If you are using SATA drive run this (if you are using mmc, change sataargs for mmcargs): <code>setenv sataargs 'setenv bootargs root=${sataroot} ${video_args} ip=dhcp'
saveenv
reset</code>
  - Leave the board boots up. Do not forget! You will not see the Linux messages in the console anymore. If you would like to see them again, you need to put the original "mmcargs" or "sataargs" value back.
  - Use Putty to connect to your board. Use the IP address from step 1 and port 22.

===== Test serial communication with Blender =====

==== Setup Serial Terminal ====

Note: If you do not have serial terminal on OpenRex, install it:
<code>apt-get install minicom setserial</code>

Open Serial terminal on OpenRex:
  - On OpenRex, run "minicom -s"
  - Go to "Serial port setup" and press A. Change the Serial Device to "/dev/ttymxc0". Press F to change Hardware Flow Control to "No". Bps / Par / Bits should be set to "11520 8N1"
  - Press "Save setup as dfl" (so next time you do not need to set it up)
  - Press "Exit" go to console
  - If you need to leave terminal, press CTRL+A and then X, Yes.

==== Prepare Blender ====

If you have not installed pySerial yet, follow these steps
  - [[https://pypi.python.org/pypi/pyserial|Download pySerial]] (needed for Blender to be able to communicate with serial port)
  - Unpack pySerial and copy "serial" directory (located in pyserial-3.2.1.tar\pyserial-3.2.1\) to c:\Program Files\Blender Foundation\Blender\2.78\python\lib\

==== Test serial communication between Blender and OpenRex ====

  - Open a new project in Blender
  - Click on "Choose screen layout" button (the button close to Help menu) and select Scripting
  - Click on "New File"
  - Copy and paste following: <sxh Python>import serial
import serial
ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM8'
ser.open()

ser.write(b'HelloWorld')

x = ser.read(size=5)
print ("OpenRex says:", x)

ser.close()</sxh>
  - Click on "Window" -> Toggle system console
  - Run the script
  - If everything is working oki, you should see HelloWorld in OpenRex minicom terminal
  - Go to OpenRex and write "works".
  - If everything works oki, in the system console you should see "OpenRex says: b'works'"

Perfect, we can communicate between Blender and OpenRex.
