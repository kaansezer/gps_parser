/*
 * BN220.c — Implementation for the BN-220 GPS receiver driver
 *
 * Copyright (c) 2025  Kaan Sezer  <kaansezer0594@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  • The above copyright notice and this permission notice shall be included
 *    in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * ---------------------------------------------------------------------------
 * @file    BN220.c
 * @author  Kaan Sezer
 * @date    27 April 2025
 * @brief   Source file providing low-level I/O, parsing routines and helper
 *          utilities for the Beitian BN-220 GPS module.  Platform-specific
 *          serial/UBX helpers should be implemented here.
 * ---------------------------------------------------------------------------
 */

#include "BN220.h"
#include <string.h>
#include <stdlib.h>

char* data [15];


int getChecksum(const char* gps_sentence) {
    if (!gps_sentence || strlen(gps_sentence) < 5)
        return 0;

    // 1) Find the '*' character and ensure that at least two hexadecimal digits follow it
    const char* star = strchr(gps_sentence, '*');
    if (!star || strlen(star) < 3)
        return 0;

    // 2) XOR range: all characters from the start of the sentence up to the ‘*’ character
    unsigned char calc_cs = 0;
    for (const char* p = gps_sentence; p < star; ++p) {
        calc_cs ^= (unsigned char)(*p);
    }

    // 3) Extract the two hexadecimal digits from the NMEA sentence
    char cs_str[3] = { star[1], star[2], '\0' };
    int nmea_cs = (int)strtol(cs_str, NULL, 16);

    // 4) Compare and keep the dest/counter section unchanged
    if (calc_cs == nmea_cs) {
        return 1;
    } else {
        return 0;
    }
}


int nmea_GLL(BN220_GPS *gps_data, char *nmea_sentence) {
    char *val[25];
    int  cnt = 0;

    // 1) Fill the val[] array with NULL
    memset(val, 0, sizeof(val));

    // 2) Keep the original sentence intact; take a copy with strdup
    char *buf = strdup(nmea_sentence);
    if (!buf) return 0;

    // 3) Split by commas: consecutive ",," must also yield an empty token ("")
    char *p     = buf;
    char *token = NULL;
    while ((token = strsep(&p, ",")) != NULL && cnt < 25) {
        val[cnt] = malloc(strlen(token) + 1);
        if (!val[cnt]) {
        	//    Clean up if malloc fails
            for (int i = 0; i < cnt; i++) free(val[i]);
            free(buf);
            return 0;
        }
        strcpy(val[cnt], token);
        cnt++;
    }

    // 4) A GLL sentence must contain at least five fields: header, lat, N/S, lon, E/W
    if (cnt < 5) {
        for (int i = 0; i < cnt; i++) free(val[i]);
        free(buf);
        return 0;
    }

    // 5) Latitude check
    char lat_ind = val[2][0];
    if (lat_ind == 'N' || lat_ind == 'S') {
    	//    Expected format: DDMM.MMMM
        char lat_d[3] = {0}, lat_m[8] = {0};
        memcpy(lat_d, val[1], 2);
        memcpy(lat_m, val[1] + 2, 6);

        int   lat_d_t = strtol(lat_d, NULL, 10);
        float lat_m_t = strtof(lat_m, NULL);
        double lat_deg = lat_d_t + lat_m_t / 60.0;

        //    Longitude check
        char lon_ind = val[4][0];
        char lon_d[4] = {0}, lon_m[8] = {0};
        memcpy(lon_d, val[3], 3);
        memcpy(lon_m, val[3] + 3, 7);

        int   lon_d_t = strtol(lon_d, NULL, 10);
        float lon_m_t = strtof(lon_m, NULL);
        double lon_deg = lon_d_t + lon_m_t / 60.0;

        // 6) Zero-value check
        if (lat_d_t == 0 || lat_m_t == 0.0f ||
            lon_d_t == 0 || lon_m_t == 0.0f) {
            for (int i = 0; i < cnt; i++) free(val[i]);
            free(buf);
            return 0;
        }

        // 7) Populate the GPS data
        gps_data->lat = lat_deg;
        gps_data->lon = lon_deg;
        gps_data->NS  = lat_ind;
        gps_data->EW  = lon_ind;
        if (val[5] && strlen(val[5]) >= 9) {
            memcpy(gps_data->lastMeasure, val[5], 9);
            gps_data->lastMeasure[9] = '\0';
        } else {
        	//    Fallback behavior, e.g., empty string
            gps_data->lastMeasure[0] = '\0';
        }

        for (int i = 0; i < cnt; i++) free(val[i]);
        free(buf);
        return 1;
    }

    //    Invalid N/S indicator
    for (int i = 0; i < cnt; i++) free(val[i]);
    free(buf);
    return 0;
}


int nmea_GSA(BN220_GPS *gps_data, char* nmea_sentence) {
    char *val[25];
    int   cnt = 0;
    memset(val, 0, sizeof(val));

    char *buf = strdup(nmea_sentence);
    if (!buf) return 0;

    char *p = buf;
    char *token;
    while ((token = strsep(&p, ",")) != NULL && cnt < 25) {
        val[cnt] = malloc(strlen(token) + 1);
        if (!val[cnt]) {
            for (int i = 0; i < cnt; i++) free(val[i]);
            free(buf);
            return 0;
        }
        strcpy(val[cnt], token);
        cnt++;
    }

    // GSA requires at least 15 fields
    if (cnt < 15) {
        for (int i = 0; i < cnt; i++) free(val[i]);
        free(buf);
        return 0;
    }

    int fix = strtol(val[2], NULL, 10);
    gps_data->fix = fix > 1 ? 1 : 0;

    int satelliteCount = 0;
    for (int i = 3; i < 15; i++) {
        if (val[i][0] != '\0') {
            satelliteCount++;
        }
    }
    gps_data->satelliteCount = satelliteCount;

    // Cleanup
    for (int i = 0; i < cnt; i++) free(val[i]);
    free(buf);
    return 1;
}

int nmea_GGA(BN220_GPS *gps_data, char* nmea_sentence) {
    char *val[25] = {0};
    int   cnt = 0;
    char *buf = strdup(nmea_sentence);
    if (!buf) return 0;

    // token
    char *p = buf, *tok;
    while ((tok = strsep(&p, ",")) && cnt < 25) {
        val[cnt++] = strdup(tok);
    }
    if (cnt < 10) goto fail;

    // --- time ---
    if (strlen(val[1]) >= 9) {
        memcpy(gps_data->lastMeasure, val[1], 9);
        gps_data->lastMeasure[9] = '\0';
    } else gps_data->lastMeasure[0] = '\0';

    // --- latitude ---
    char lat_ind = val[3][0];              // **val[3]**
    if (lat_ind != 'N' && lat_ind != 'S') goto fail;
    char lat_d[3] = {0}, lat_m[8] = {0};
    memcpy(lat_d, val[2], 2);              // **val[2]**
    memcpy(lat_m, val[2] + 2, 6);
    int   lat_deg_i = strtol(lat_d, NULL, 10);
    float lat_deg_f = strtof(lat_m, NULL) / 60.0f;
    double lat = lat_deg_i + lat_deg_f;
    if (lat <= 0 || lat >= 90) goto fail;

    // --- longitude ---
    char lon_ind = val[5][0];              // **val[5]**
    if (lon_ind != 'E' && lon_ind != 'W') goto fail;
    char lon_d[4] = {0}, lon_m[8] = {0};
    memcpy(lon_d, val[4], 3);              // **val[4]**
    memcpy(lon_m, val[4] + 3, 7);
    int   lon_deg_i = strtol(lon_d, NULL, 10);
    float lon_deg_f = strtof(lon_m, NULL) / 60.0f;
    double lon = lon_deg_i + lon_deg_f;
    if (lon <= 0 || lon >= 180) goto fail;

    // --- indicator ---
    gps_data->lat = (lat_ind == 'S' ? -lat : lat);
    gps_data->lon = (lon_ind == 'W' ? -lon : lon);
    gps_data->fix = (strtol(val[6], NULL, 10) > 0);
    gps_data->satelliteCount = strtol(val[7], NULL, 10);
    gps_data->hdop = strtof(val[8], NULL) ?: gps_data->hdop;
    gps_data->altitude = strtof(val[9], NULL) ?: gps_data->altitude;
    for (int i = 0; i < cnt; i++) free(val[i]);
    free(buf);
    return 1;

fail:
    for (int i = 0; i < cnt; i++) free(val[i]);
    free(buf);
    return 0;
}


int nmea_GSV(BN220_GPS *gps_data, char* nmea_sentence) {
    char *val[25];
    int   cnt = 0;
    memset(val, 0, sizeof(val));
    char *buf = strdup(nmea_sentence);
    if (!buf) return 0;
    char *p = buf;
    char *token;
    while ((token = strsep(&p, ",")) != NULL && cnt < 25) {
        val[cnt] = malloc(strlen(token) + 1);
        if (!val[cnt]) {
            for (int i = 0; i < cnt; i++) free(val[i]);
            free(buf);
            return 0;
        }
        strcpy(val[cnt], token);
        cnt++;
    }
    if (cnt < 2) {
        for (int i = 0; i < cnt; i++) free(val[i]);
        free(buf);
        return 0;
    }
    for (int i = 0; i < cnt; i++) free(val[i]);
    free(buf);
    return 1;
}

void gpsParse(BN220_GPS *gps_data, uint8_t *buffer){
	memset(data,0,sizeof(data));
	char* sentence = strtok(buffer,"$");
	int cnt =0;
	while(sentence != NULL){
        data[cnt++] = malloc(strlen(sentence)+1);
        strcpy(data[cnt-1], sentence);
        sentence = strtok(NULL, "$");
	}
	for(int i=0;i<cnt;i++){
		if(strstr(data[i], "\r\n")!=NULL && getChecksum(data[i])){
			if(strstr(data[i], "GLL")!=NULL){
				nmea_GLL(gps_data, data[i]);
			}
			else if(strstr(data[i], "GSA")!=NULL){
				nmea_GSA(gps_data, data[i]);
			}
			else if(strstr(data[i], "GGA")!=NULL){
				nmea_GGA(gps_data, data[i]);
			}
			else if(strstr(data[i], "GSV")!=NULL){
				nmea_GSV(gps_data, data[i]);
			}
		}
	}
	for(int i = 0; i<cnt; i++)
		free(data[i]);
}
