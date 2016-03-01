#ifndef TWO_TELOSB_H
#define TWO_TELOSB_H

typedef nx_struct TwoTelosbMsg
{
	nx_uint16_t NodeId;
	nx_uint16_t Temp;
	nx_uint16_t Lumi;
	nx_uint16_t Humid;
	
	
} TwoTelosbMsg_t;

enum
{
	AM_RADIO = 6
};

#endif /* TWO_TELOSB_H */
