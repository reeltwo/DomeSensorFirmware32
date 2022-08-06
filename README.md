# Dome Sensor Ring Firmware

This is the sketch for the dome sensor ring board.

## Libraries Used

<ul>
<li>https://github.com/reeltwo/Reeltwo</li>
</ul>

## Serial Configuration commands

`#DPZERO`:
Factory reset all settings in EEPROM
*Examples*:

    #DPZERO

`#DPCONFIG`:
List all settings in EEPROM
*Examples*:

    #DPCONFIG

`#DPBAUD`[number]:
Specify sensor ring baud rate (default 57600)
*Examples*:

    #DPBAUD57600 (set baud rate to 57600)

`#DPMIN`[number]:
Specify sensor ring minimum analog reference value (default 0)
*Examples*:

    #DPMIN0 (set minimum analog reference to 0)

`#DPMAX`[number]:
Specify sensor ring maximum analog reference value (default 1023)
*Examples*:

    #DPMAX1023 (set maximum analog reference to 1023)
