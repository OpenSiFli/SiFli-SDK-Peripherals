#ifndef RTCONFIG_PROJECT_H__
#define RTCONFIG_PROJECT_H__

#if defined(_MSC_VER)
    #define RT_HEAP_SIZE   (680000)
    #define NORESOURCE
    #define _CRT_ERRNO_DEFINED
    #define _INC_WTIME_INL
    #define _INC_TIME_INL

    #pragma warning(disable:4273)
    #pragma warning(disable:4312)
    #pragma warning(disable:4311)
    #pragma warning(disable:4996)
    #pragma warning(disable:4267)
    #pragma warning(disable:4244)
#endif

#endif
