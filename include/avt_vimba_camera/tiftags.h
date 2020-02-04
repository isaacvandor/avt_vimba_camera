/*
 * prosilicaDrvTags.h
 *
 *  Created on: Jul 26, 2011
 *      Author: ivaughn
 */

#ifndef PROSILICADRVTAGS_H_
#define PROSILICADRVTAGS_H_

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

#define PROTIFFTAG_BASE                       60100
// housekeeping
#define PROTIFFTAG_MAGIC                   (PROTIFFTAG_BASE+0)
#define PROTIFFTAG_VERSION                 (PROTIFFTAG_BASE+1)

#define PROTIFFTAG_MAGIC_VALUE             ((uint32_t)0x807D05D7)

// version 0 was no-tags, and encoded all data in the filename
// version 1 was ASCII-tiff tags
#define PROTIFFTAG_VERSION_2               ((uint32_t)2)
#define PROTIFFTAG_VERSION_3               ((uint32_t)3)
#define PROTIFFTAG_CURRENT_VERSION         PROTIFFTAG_VERSION_3
// legacy tags
#define PROTIFFTAG_BINNINGHORIZONTAL       (PROTIFFTAG_BASE+10)
#define PROTIFFTAG_BINNINGVERTICAL         (PROTIFFTAG_BASE+11)
#define PROTIFFTAG_EXPOSUREAUTOTARGET      (PROTIFFTAG_BASE+12)
#define PROTIFFTAG_EXPOSUREAUTO            (PROTIFFTAG_BASE+13)
#define PROTIFFTAG_EXPOSURETIMEABS         (PROTIFFTAG_BASE+14)
#define PROTIFFTAG_ACQUISITIONFRAMERATEABS (PROTIFFTAG_BASE+15)
#define PROTIFFTAG_GAIN                    (PROTIFFTAG_BASE+16)
#define PROTIFFTAG_HEIGHT                  (PROTIFFTAG_BASE+17)
#define PROTIFFTAG_OFFSETX                 (PROTIFFTAG_BASE+18)
#define PROTIFFTAG_OFFSETY                 (PROTIFFTAG_BASE+19)
#define PROTIFFTAG_STATFRAMERATE           (PROTIFFTAG_BASE+20)
#define PROTIFFTAG_STATFRAMEDELIVERED      (PROTIFFTAG_BASE+21)
#define PROTIFFTAG_STATFRAMEDROPPED        (PROTIFFTAG_BASE+22)
#define PROTIFFTAG_STREAMBYTESPERSECOND    (PROTIFFTAG_BASE+23)
#define PROTIFFTAG_WIDTH                   (PROTIFFTAG_BASE+24)
#define PROTIFFTAG_NIR_MODE                (PROTIFFTAG_BASE+25)

// image info
#define PROTIFFTAG_UTIME_LO                (PROTIFFTAG_BASE+30)
#define PROTIFFTAG_UTIME_HI                (PROTIFFTAG_BASE+31)
#define PROTIFFTAG_IMG_NUM                 (PROTIFFTAG_BASE+32)
#define PROTIFFTAG_IMG_FILENAME            (PROTIFFTAG_BASE+33)
#define PROTIFFTAG_IMG_SURVEYID            (PROTIFFTAG_BASE+34)

static const ProTiffFieldInfo[]={
        { PROTIFFTAG_MAGIC,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"MagicNumber" },
        { PROTIFFTAG_VERSION,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"Version" },
        { PROTIFFTAG_BINNINGHORIZONTAL,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"BinningHorizontal" },
        { PROTIFFTAG_BINNINGVERTICAL,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"BinningVertical" },
        { PROTIFFTAG_EXPOSUREAUTOTARGET,  1, 1,	TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"ExposureAutoTarget" },
        { PROTIFFTAG_EXPOSUREAUTO,	-1, -1,		TIFF_ASCII,	FIELD_CUSTOM,
                1,	0,	(char*)"ExposureAuto" },
        { PROTIFFTAG_EXPOSURETIMEABS,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"ExposureTimeAbs" },
        { PROTIFFTAG_ACQUISITIONFRAMERATEABS,	1, 1,		TIFF_FLOAT,	FIELD_CUSTOM,
                1,	0,	(char*)"AcquisitionFrameRateAbs" },
        { PROTIFFTAG_GAIN,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"Gain" },
        { PROTIFFTAG_HEIGHT,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"Height" },
        { PROTIFFTAG_OFFSETX,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"OffsetX" },
        { PROTIFFTAG_OFFSETY,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"OffsetY" },
        { PROTIFFTAG_STATFRAMERATE, 1, 1,		TIFF_FLOAT,	FIELD_CUSTOM,
                1,	0,	(char*)"StatFrameRate" },
        { PROTIFFTAG_STATFRAMEDELIVERED,   1, 1,	    TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"StatFrameDelivered" },
        { PROTIFFTAG_STATFRAMEDROPPED,    1, 1,	   TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"StatFrameDropped" },
        { PROTIFFTAG_STREAMBYTESPERSECOND, 1, 1,	    TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"StreamBytesPerSecond" },
        { PROTIFFTAG_WIDTH,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"Width" },
        { PROTIFFTAG_NIR_MODE,	1, 1,		TIFF_ASCII,	FIELD_CUSTOM,
                1,	0,	(char*)"NirMode" },
        { PROTIFFTAG_UTIME_LO,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"UtimeLo" },
        { PROTIFFTAG_UTIME_HI,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"UtimeHi" },
        { PROTIFFTAG_IMG_NUM,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	(char*)"ImgNum" },
        { PROTIFFTAG_IMG_FILENAME,	1, 1,		TIFF_ASCII,	FIELD_CUSTOM,
                1,	0,	(char*)"ImgFilename" },
        { PROTIFFTAG_IMG_SURVEYID,	1, 1,		TIFF_ASCII,	FIELD_CUSTOM,
                1,	0,	(char*)"SurveyId" },
};

#define PROTIFFTAG_N (sizeof(ProTiffFieldInfo)/sizeof(ProTiffFieldInfo[0]))

#ifdef __cplusplus
}
#endif

#endif /* PROSILICADRVTAGS_H_ */