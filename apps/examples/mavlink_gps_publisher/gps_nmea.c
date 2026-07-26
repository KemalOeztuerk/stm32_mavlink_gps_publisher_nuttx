#include "gps_nmea.h"
#include "nav_state.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

#define GPS_LINE_MAX      96U
#define GPS_MAX_FIELDS    16U

static int s_fd = -1;

/* Combined fix state: RMC and GGA each supply a subset of fields, so we keep
 * the last known value of each and republish after every parsed sentence. */
static bool s_fix_valid;
static uint8_t s_fix_type;
static int32_t s_lat_degE7;
static int32_t s_lon_degE7;
static float s_alt_m;
static float s_speed_mps;
static float s_course_deg;
static uint8_t s_satellites_visible;
static float s_hdop;

int GPSNMEA_Init(const char *devpath)
{
    int fd = open(devpath, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) == 0) {
        cfsetspeed(&tio, 9600);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        tcsetattr(fd, TCSANOW, &tio);
    }

    s_fd = fd;
    return 0;
}

static bool sentence_is(const char *field0, const char *suffix)
{
    size_t flen = strlen(field0);
    size_t slen = strlen(suffix);
    if (flen < slen) {
        return false;
    }
    return strcmp(field0 + (flen - slen), suffix) == 0;
}

/* NMEA lat/lon are "d..ddmm.mmmm" (degrees + decimal minutes); works for both
 * 2-digit (lat) and 3-digit (lon) degree widths since it only divides by 100. */
static int32_t parse_nmea_coord(const char *raw, char hemisphere, char negative_hemisphere)
{
    if (raw[0] == '\0') {
        return 0;
    }
    float value = strtof(raw, NULL);
    float degrees = (float)((int)(value / 100.0f));
    float minutes = value - degrees * 100.0f;
    float decimal = degrees + minutes / 60.0f;
    if (hemisphere == negative_hemisphere) {
        decimal = -decimal;
    }
    return (int32_t)(decimal * 1.0e7f);
}

static void publish_fix(void)
{
    NavState_UpdateGps(s_fix_valid, s_fix_type, s_lat_degE7, s_lon_degE7,
                        s_alt_m, s_speed_mps, s_course_deg,
                        s_satellites_visible, s_hdop, NAV_GPS_SOURCE_NMEA);
}

static void parse_gprmc(char *fields[], int n)
{
    /* $--RMC,time,status,lat,N/S,lon,E/W,speed_knots,course,date,... */
    if (n < 9) {
        return;
    }
    s_fix_valid = (fields[2][0] == 'A');
    s_lat_degE7 = parse_nmea_coord(fields[3], fields[4][0], 'S');
    s_lon_degE7 = parse_nmea_coord(fields[5], fields[6][0], 'W');
    s_speed_mps = strtof(fields[7], NULL) * 0.514444f; /* knots -> m/s */
    s_course_deg = strtof(fields[8], NULL);
    publish_fix();
}

static void parse_gpgga(char *fields[], int n)
{
    /* $--GGA,time,lat,N/S,lon,E/W,fixquality,numSV,HDOP,alt,M,... */
    if (n < 10) {
        return;
    }
    uint8_t fix_quality = (uint8_t)atoi(fields[6]);
    s_fix_type = (fix_quality > 0U) ? 3U : 1U;
    s_lat_degE7 = parse_nmea_coord(fields[2], fields[3][0], 'S');
    s_lon_degE7 = parse_nmea_coord(fields[4], fields[5][0], 'W');
    s_satellites_visible = (uint8_t)atoi(fields[7]);
    s_hdop = strtof(fields[8], NULL);
    s_alt_m = strtof(fields[9], NULL);
    publish_fix();
}

static void parse_nmea_line(char *line)
{
    char *checksum_marker = strchr(line, '*');
    if (checksum_marker != NULL) {
        *checksum_marker = '\0';
    }

    char *fields[GPS_MAX_FIELDS];
    int nfields = 0;
    char *p = line;
    while (nfields < (int)GPS_MAX_FIELDS) {
        fields[nfields++] = p;
        char *comma = strchr(p, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    if (nfields == 0 || fields[0][0] != '$') {
        return;
    }

    if (sentence_is(fields[0], "RMC")) {
        parse_gprmc(fields, nfields);
    } else if (sentence_is(fields[0], "GGA")) {
        parse_gpgga(fields, nfields);
    }
}

void *gps_task(void *argument)
{
    (void)argument;
    static char line[GPS_LINE_MAX];
    uint32_t idx = 0;

    for (;;) {
        uint8_t byte;
        ssize_t n = read(s_fd, &byte, 1);
        if (n != 1) {
            continue;
        }
        if (byte == '\n') {
            line[idx] = '\0';
            parse_nmea_line(line);
            idx = 0;
        } else if (byte != '\r') {
            if (idx < GPS_LINE_MAX - 1U) {
                line[idx++] = (char)byte;
            } else {
                idx = 0; /* malformed/overlong line, resync */
            }
        }
    }

    return NULL;
}
