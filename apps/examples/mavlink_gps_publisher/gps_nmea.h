#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opens the GPS UART device (the M8N link) at 9600 8N1 and keeps the fd
 * for gps_task() to read from. Call once from main before spawning
 * gps_task(). Returns 0 on success, -1 on failure (errno set). */
int GPSNMEA_Init(const char *devpath);

/* pthread entry: assembles NMEA lines and parses GPRMC/GGA into nav_state. */
void *gps_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* GPS_NMEA_H */
