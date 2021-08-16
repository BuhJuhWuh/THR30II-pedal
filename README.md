# THRII-direct-USB-pedalboard
A pedalboard that can send complete settings patches to a THRII guitar amplifier with direct USB-connection

Demonstration video:
https://youtu.be/Kstgtiw6ibM

The Yamaha THRII-series is a guitar practice amplifier.

There is a smartphone app and a PC and Mac application to control all the sound parameters.
There even is a bluetooth connection for using foot switches with this amp. But you always need the smartphone app running to use a bluetooth foot switch.
Furthermore you can only switch some dedicated parameters with a bluetooth footswitch.

My project's goal is to avoid the sometimes not very stable bluetooth connection and connect directly to the amp via USB-cable.
It is possible to send whole "patch" settings including a complete set of parameters by pressing a button on the board with your foot.
(see demonstration video)

To achieve this, a Teensy 3.6 or 4.1 microcontroller board, that is capable of USB-host mode, connects to the THRII via the wirde USB interface.
It then activates the USB-MIDI-interface and communicates with the THRII by exchangig MIDI SystemExclusive messages.

On a TFT display the actual settings are shown and with foot switches a "patch" can be pre-selected from a library stored on the controller or a SD-card.

Patches can be pre-selected by choosing a group of 5 patches (e.g. containing 5 sounds for a song) and separately choosing a patch from that group. 
A "Patch-Send" button activates that special patch on the THRII.

A "SOLO" button either activates an altered sound patch fitting to the actual one or (if no dedicated solo patch exists) toggles a volume increasement.

