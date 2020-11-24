# RR-duino

This sketch enables an arduino to control sensors (I/O) and turnouts (using servomotors) on a railroad layout.

It has been tested with mega, leonardo but any other should work, maybe you have to disable debug as it uses another serial port.

The typical setup would be: several arduinos running this sketch on a "bus" (rs485 for example) as slaves and a master node to do all the requests. You can check openlcb_gateway, it is a openlcb gateway written in Python that can handle RR_duino nodes so they appear as regular openlcb nodes with ID/events, CDI configuration,... Soon available: a direct connection to JMRI (using jython) to control the nodes from a master node via WiFi or ethernet (ESP-8266 for example).

Features:

- every	arduino ("node") has its own address that can be configured online, no need to change the sketch. Each sensor/turnout has its own subadress, you can have 63 sensors and 63 turnouts max.

- All configuration (pins and I/O for sensors, pins to control servo and optional relays for turnouts (relays are used in general to power the frog turnout) is done on the wire thanks to the protocol. Everything can be changed on the fly.

- All configuration can be saved to eeprom; the protocol allows for enumerating each device config (pins...)

- All outputs (sensors and turnouts positions) can be saved to eeprom; they will be restored automatically when loading the config from eeprom.

- All inputs changes will generate an "async event"; inputs mean: input sensors obviously (train detection for example), but also turnouts as they generate events when they reach their final position.

- Turnouts position (straight or thrown) can be configured via the protocol. The protocol also has a "position fine tuning" so you don't have to precisely mount the servo (see below)

- Protocol is very lightweight (a lot of commands/answer are 4 bytes long), you can poll all sensors/turnouts values at once, you will get a bit packed answer.

- you can set more turnouts than the maximum number of servos the arduino can handle (typically 12 for a uno/leonardo and 48 for a mega) as a "pool" of servos is used to work around this limitation. The pool size can be changed (though you have to modify a constant in the turnouts.h, NB_SERVOS, max is 12 for uno (servo library limit) and 48 for a mega for example. Make sure your power supply can handle that many servos moving at the same time.

Note for the servo control of a turnout: just set the servo to 90° position before mounting it; mount it on the turnout (no need to set the extreme position, it does not matter if the turnout is not straight or thrown). Then use the fine tuning command to find the precise positions for straight/thrown, and then config the servo positions accordingly.

# RR_duino_bus_controller

 This sketch is used to control a "bus" (read rs485 bus here) of RR-duino nodes. For now this sketch is made for esp8266 boards but may be adapted to any wifi enabled arduino compatible easily (needs some ifdef basically)
It will ping nodes to set/get new states, wake dead nodes up if necessary and report to a server (see below)

# jmri-RR-duino

Several python scripts:

- pico-rr-duino-setup.py: enables you to setup all your sensors/turnouts, test them, fine tune the servo positions and backup restore nodes from files (you can backup a node and restore it as it was in case of an hardware failure for example) via a text user interface (see the wiki page).

- RR_duino_jmri_monitor.py: small python server that can connect to esp8266s running RR_duino_bus_controller (see above). This server will send all necessary information to a jython script to make the link with JMRI. It will: create all sensors/turnouts necessary in JMRI, translate commands to and from JMRI to command to and from bus controllers. This is all networked so this script can run wherever you want (does not have to run on the same computer as JMRI).

- JMRI_RR_duino.py: jython script to run in JMRI that will receive necessary information from RR_duino_jmri_monitor.py from the RR_duino busses.
