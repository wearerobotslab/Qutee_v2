#ifndef UDP_LOG_HPP
#define UDP_LOG_HPP

// Send the ESP-IDF logs over WiFi (UDP) to the micro-ROS agent's computer, in addition to USB.
// Read them with software/tools/wifi_monitor.py. Port: QUTEE settings -> CONFIG_QUTEE_LOG_UDP_PORT.

// Call first in app_main: from then on, logs are also kept in a buffer (the boot logs included)
void udp_log_init();

// Call once WiFi is connected: sends the buffered logs, then every new one
void udp_log_start();

#endif
