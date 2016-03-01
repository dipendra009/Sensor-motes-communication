#ifndef LIGHT_TEMP_H
#define LIGHT_TEMP_H

typedef nx_struct radio_sense_msg {
  nx_uint16_t error;
  nx_uint16_t temp;
  nx_uint16_t light;
  
} radio_sense_msg_t;

enum {
  AM_RADIO_SENSE_MSG = 7,
};

#endif
