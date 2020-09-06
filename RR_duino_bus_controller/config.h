#ifndef CONFIG_H
#define CONFIG_H

//Communications settings
#define TO_BUS Serial
#define STATUS_LED LED_BUILTIN
#define SERIAL_SPEED 38400
#define DATA_DIR_PIN 255  //set it to a pin number if bus if half duplex and need to set data dir through this pin
                        //set it to 255 if bus is full duplex and this is not needed

#define WIFISSID "YOUR SSID"
#define WIFIPASS "YOUR PASSWD"

#define SERVER_IP "192.168.0.22"
#define SERVER_PORT 50011

#define DEBUG_SERVER_IP "192.168.0.22"
#define DEBUG_SERVER_PORT 50000

//Bus settings
#define BUS_NUMBER 1
#endif // CONFIG_H
