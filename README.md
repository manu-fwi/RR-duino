# RR-duino

TODO:

- reset turnout/output sensors on startup to last kown position
- make sure we save turnout positions and output sensors value to eeprom on every change
- add an isr called every 10 ms to take care of pulse relay pins (long answers can hog the cpu for longer than 10ms I thin) and to move servos (probablu need to use timer2 to avoid conflicts with millis() and the servo library)
