#Release Notes:


##MP SiK 2.5:

###Alterations
* Fixed issue where setting power to zero would not be persistent on reboot
* Change onboard temperature reading calculation to be within +/-5 DegC by using manufacturer register.

###NEW FEATURES!!
* Adding support for new rfd900p (plus) radio modem
* Added AES encryption when using new RFD900u and RFD900p boards

----
##MP SiK 2.4:
###Improvements
* Altered timing for better throughput
* Removed support for MAVLink 0.9 support freeing up code space
* Updated Config to use a CRC instead of the XOR 

----

##MP SiK 2.3:

###Bug Fixes
1. Watchdog turned off due to odd behaviour
2. RTI5 returns full list of commands (Watchdog problem)
3. RT&W should be stable now (Watchdog problem)
4. Tweeking Radio timing to improve preformace

###NEW FEATURES!!

1. ATI8 Returns if the node is sync'd with the base (Does not work on base node)

2. Users can now controll unused pins. This can be preformed by the following commands

Command       | Function | Description
------------- | ---------|-------------
ATPP          | Print    | Print All Pins Settings
ATPI=1        | Input    | Set Pin 1 to Input
ATPR=1        | Read     | Read Pin 1 value (When set to input)
ATPO=2        | Output   | Set Pin 2 to Output (Output's by Default can only be controlled by AT cmd)
ATPC=2,1      | Control  | Turn pin 2 on  - Output Mode / Set internal pull up resistor - Input Mode 
ATPC=2,0      | Control  | Turn pin 2 off - Output Mode / Set internal pull down resistor - Input Mode

Pins can't be maped between modems, any reading or writing of pin state has to be done using AT or RT commands at this stage. 

Mapping between the pin numbers above and the port number are below

######RFD900
Pin  | Port
---- | ----
0    | 2.3
1    | 2.2
2    | 2.1
3    | 2.0
4    | 2.6
5    | 0.1

######RFD900u
Pin  | Port
---- | ----
0    | 1.0
1    | 1.1

----
##MP SiK 2.2:
###Lookup Changes

----
##MP SiK 2.1:
###First Release SiK MultiPoint Firmware

###NEW FEATURES!!
1. Support for Multinode Networks (more than 2!)
2. Improved Radio stack to minimise packet delays
