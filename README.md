# Quango
![Image](https://raw.githubusercontent.com/dchwebb/Quango/master/Graphics/Quango_Panel.jpg "icon")

Quango
--------

Quango is a combined MIDI interface, VCA and envelope for controlling four voice polyphonic Eurorack Modular Oscillators. Polyphonic signal interconnections are made via RJ-45 cables (ie Ethernet cables).

Quango can control two VCOs simultaneously, supplying 1V/Octave pitch information and then shaping the returned audio using 8 ADSR envelopes controlling VCAs for each voice.

The MIDI interface receives both USB and serial MIDI data and also features a calibration mechanism where the pitch CV for each voice is adjusted to correct any tuning errors.

Each of the two channels uses an RJ-45 connector to send CV data on lines 1-4 and receive audio data on lines 5-8. The received audio data is used both for pitch calibration and sent into the ADSR-controlled VCAs.

Both channel voices are mixed and output onto an RJ-45 connection. Additionally each channel's four voices can be separately output on a header on the back of the unit. the gate signal for each voice is also output on an RJ-45 for controlling external envelopes etc.

Additional controls include an octave switch to allow channel B's CV range to be adjusted up or down an octave and a fader to control the overall level of each voice. LEDs show the current level of the VCA of each voice for each channel.


Technical
---------

Quango uses an STM32G473 microcontroller which handles the MIDI interfacing, generates 12-bit DAC signals to control the envelopes, reads the front panel controls from its internal ADCs and controls two external DACs for generating the pitch CV and one envelope (the microcontroller has 7 internal 12 bit DACs so the 8th envelope is supplied from an external 12 bit DAC).

![Image](https://raw.githubusercontent.com/dchwebb/Quango/master/Graphics/Quango_PCB.jpg "icon")

The 1V/Octave Pitch output is generated using an SPI controlled 16-bit 8 channel Analog Devices AD5676 DAC. A Microchip MCP48 12 bit DAC outputs the one envelope generated externally to the microcontroller. Alfa AS3364 quad linear VCAs are used for independent level control of each channel.

A PS508 audio multiplexer switches the 8 audio input channels to a single ADC channel on the microcontroller for pitch detection. Output level LEDs are controlled via PWM from the microcontroller.

Construction is a sandwich of three PCBs with a component board, a controls board and a panel. PCBs designed in Kicad v6.

[Components schematic](https://raw.githubusercontent.com/dchwebb/Quango/master/Hardware/Quango_Components.pdf)

[Controls schematic](https://raw.githubusercontent.com/dchwebb/Quango/master/Hardware/Quango_Controls.pdf)

In addition the the Eurorack +/-12V rails (which have polarity protection and filtering) a 5v rail is supplied using an AMS1117-5.0 LDO. A 3.3v rail is generated with an LM1117-3.3.

Errata
------

Version 1 of the component PCB had the power supply connections to op-amp U14 reversed.