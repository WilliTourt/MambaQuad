#include "ATGM336H.h"

ATGM336H::ATGM336H(void)
{
    nmea_parser_init(&_parser);
    nmea_zero_INFO(&_info);
}


int ATGM336H::update(uint8_t* data)
{
    int nread = 0;
    nread = nmea_parse(&_parser, (const char*)data, strlen((const char*)data), &_info);
    GMTconvert(&_info.utc, &_beiJingTime, 8, 1);
    
    return nread;
}








