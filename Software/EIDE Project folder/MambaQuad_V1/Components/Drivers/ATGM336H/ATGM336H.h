#ifndef __ATGM336H_H__
#define __ATGM336H_H__

#include "cpp_main.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "./nmeaLib/nmea.h"
#include "string.h"
#include "math.h"

#ifdef __cplusplus
}
#endif


class ATGM336H
{
    public:
        ATGM336H(void);

        int update(uint8_t* data);


        inline int get_smask(void) { return _info.smask; }
        inline int get_sig(void) { return _info.sig; }
        inline int get_fix(void) { return _info.fix; }
        inline char get_mode(void) { return _info.mode; }

        inline double get_PDOP(void) { return _info.PDOP; }
        inline double get_HDOP(void) { return _info.HDOP; }
        inline double get_VDOP(void) { return _info.VDOP; }

        inline int get_satinfo_inuse(void) { return _info.satinfo.inuse; }
        inline int get_satinfo_inview(void) { return _info.satinfo.inview; }
        inline int get_BDsatinfo_inuse(void) { return _info.BDsatinfo.inuse; }
        inline int get_BDsatinfo_inview(void) { return _info.BDsatinfo.inview; }

        inline double get_lat(void) { return _info.lat; }
        inline double get_lon(void) { return _info.lon; }
        inline double get_elv(void) { return _info.elv; }
        inline double get_speed(void) { return _info.speed; }
        inline double get_sog(void) { return _info.sog; }
        inline double get_direction(void) { return _info.direction; }
        inline double get_declination(void) { return _info.declination; }

        inline int get_UTC_year(void) { return _info.utc.year; }
        inline int get_UTC_month(void) { return _info.utc.mon; }
        inline int get_UTC_day(void) { return _info.utc.day; }
        inline int get_UTC_hour(void) { return _info.utc.hour; }
        inline int get_UTC_minute(void) { return _info.utc.min; }
        inline int get_UTC_second(void) { return _info.utc.sec; }
        inline int get_UTC_hsecond(void) { return _info.utc.hsec; }

        inline int get_BeiJing_year(void) { return _beiJingTime.year; }
        inline int get_BeiJing_month(void) { return _beiJingTime.mon; }
        inline int get_BeiJing_day(void) { return _beiJingTime.day; }
        inline int get_BeiJing_hour(void) { return _beiJingTime.hour; }
        inline int get_BeiJing_minute(void) { return _beiJingTime.min; }
        inline int get_BeiJing_second(void) { return _beiJingTime.sec; }
        inline int get_BeiJing_hsecond(void) { return _beiJingTime.hsec; } 

    private:
        nmeaPARSER _parser;
        nmeaINFO _info;
        nmeaTIME _beiJingTime;


};






#endif /* __ATGM336H_H__ */
