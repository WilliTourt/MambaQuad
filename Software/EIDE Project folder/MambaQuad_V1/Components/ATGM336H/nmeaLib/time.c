/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 */

/*! \file time.h */

#include "time.h"

#ifdef NMEA_WIN
#   pragma warning(disable: 4201)
#   pragma warning(disable: 4214)
#   pragma warning(disable: 4115)
#   include <windows.h>
#   pragma warning(default: 4201)
#   pragma warning(default: 4214)
#   pragma warning(default: 4115)
#else
#   include <time.h>
#endif





#ifdef NMEA_WIN

void nmea_time_now(nmeaTIME *stm)
{
    SYSTEMTIME st;

    GetSystemTime(&st);

    stm->year = st.wYear - 1900;
    stm->mon = st.wMonth - 1;
    stm->day = st.wDay;
    stm->hour = st.wHour;
    stm->min = st.wMinute;
    stm->sec = st.wSecond;
    stm->hsec = st.wMilliseconds / 10;
}

#else /* NMEA_WIN */

//modify by fire 这个函数用于产生系统时间,可以通过RTC为它设置时间，现在设置为默认值
void nmea_time_now(nmeaTIME *stm)
{
//    time_t lt ;
//   struct tm *tt;

    //time(&lt);          //modify by fire 
   // tt = gmtime(&lt);

//    stm->year = tt->tm_year;
//    stm->mon = tt->tm_mon;
//    stm->day = tt->tm_mday;
//    stm->hour = tt->tm_hour;
//    stm->min = tt->tm_min;
//    stm->sec = tt->tm_sec;
//    stm->hsec = 0;
    stm->year = 2026;
    stm->mon =  0;
    stm->day = 1;
    stm->hour = 0;
    stm->min = 0;
    stm->sec = 0;
    stm->hsec = 0;
}

/******************************************************************************************************** 
**     函数名称:            bit        IsLeapYear(uint8_t    iYear) 
**    功能描述:            判断闰年(仅针对于2000以后的年份) 
**    入口参数：            iYear    两位年数 
**    出口参数:            uint8_t        1:为闰年    0:为平年 
********************************************************************************************************/ 
static int IsLeapYear(char iYear) 
{ 
    int    Year; 
    Year    =    2000+iYear; 
    if((Year&3)==0) 
    { 
        return ((Year%400==0) || (Year%100!=0)); 
    } 
     return 0; 
} 

/******************************************************************************************************** 
**     函数名称:            void    GMTconvert(uint8_t *DT,uint8_t GMT,uint8_t AREA) 
**    功能描述:            格林尼治时间换算世界各时区时间 
**    入口参数：            *DT:    表示日期时间的数组 格式 YY,MM,DD,HH,MM,SS 
**                        GMT:    时区数 
**                        AREA:    1(+)东区 W0(-)西区 
********************************************************************************************************/ 
void GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, char GMT,char AREA) 
{ 
    int    YY,MM,DD,hh,mm,ss;        //年月日时分秒暂存变量 
     
    if(GMT==0)    return;                //如果处于0时区直接返回 
    if(GMT>12)    return;                //时区最大为12 超过则返回         

    YY    =    SourceTime->year;                //获取年 
    MM    =    SourceTime->mon;                 //获取月 
    DD    =    SourceTime->day;                 //获取日 
    hh    =    SourceTime->hour;                //获取时 
    mm    =    SourceTime->min;                 //获取分 
    ss    =    SourceTime->sec;                 //获取秒 

    if(AREA)                        //东(+)时区处理 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//如果与格林尼治时间处于同一天则仅加小时即可 
        else                        //如果已经晚于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+GMT-24;        //先得出时间 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //大月份(12月单独处理) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //小月份2月单独处理) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //处理2月份 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //本来是闰年且是2月29日 或者不是闰年且是2月28日 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //处理12月份 
            { 
                if(DD<31)    DD++; 
                else        //跨年最后一天 
                {               
                    DD    =    1; 
                    MM    =    1; 
                    YY    ++; 
                } 
            } 
        } 
    } 
    else 
    {     
        if(hh>=GMT)    hh    -=    GMT;    //如果与格林尼治时间处于同一天则仅减小时即可 
        else                        //如果已经早于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+24-GMT;        //先得出时间 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //上月是大月份(1月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //上月是小月份2月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //处理上个月是2月份 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //不是闰年 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //处理1月份 
            { 
                if(DD>1)    DD--; 
                else        //新年第一天 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //更新年 
    ConvertTime->mon    =    MM;                //更新月 
    ConvertTime->day    =    DD;                //更新日 
    ConvertTime->hour   =    hh;                //更新时 
    ConvertTime->min    =    mm;                //更新分 
    ConvertTime->sec    =    ss;                //更新秒 
}  


#endif
