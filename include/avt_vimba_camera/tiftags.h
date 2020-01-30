//
// Created by ivandor on 1/30/20.
//

#ifndef TIFTAGS_H
#define TIFTAGS_H

#include <stdint.h>

#define PROTIFFTAG_BASE                       60100
// housekeeping
#define PROTIFFTAG_MAGIC                   (PROTIFFTAG_BASE+0)
#define PROTIFFTAG_VERSION                 (PROTIFFTAG_BASE+1)

#define PROTIFFTAG_MAGIC_VALUE             ((uint32_t)0x807D05D7)

// version 0 was no-tags, and encoded all data in the filename
// version 1 was ASCII-tiff tags
#define PROTIFFTAG_VERSION_2               ((uint32_t)2)
#define PROTIFFTAG_CURRENT_VERSION         PROTIFFTAG_VERSION_2
// legacy tags
#define PROTIFFTAG_BINNINGX                (PROTIFFTAG_BASE+10)
#define PROTIFFTAG_BINNINGY                (PROTIFFTAG_BASE+11)
#define PROTIFFTAG_EXPOSUREAUTOTARGET      (PROTIFFTAG_BASE+12)
#define PROTIFFTAG_EXPOSUREMODE            (PROTIFFTAG_BASE+13)
#define PROTIFFTAG_EXPOSUREVALUE           (PROTIFFTAG_BASE+14)
#define PROTIFFTAG_FRAMERATE               (PROTIFFTAG_BASE+15)
#define PROTIFFTAG_GAINVALUE               (PROTIFFTAG_BASE+16)
#define PROTIFFTAG_HEIGHT                  (PROTIFFTAG_BASE+17)
#define PROTIFFTAG_REGIONX                 (PROTIFFTAG_BASE+18)
#define PROTIFFTAG_REGIONY                 (PROTIFFTAG_BASE+19)
#define PROTIFFTAG_STATFRAMERATE           (PROTIFFTAG_BASE+20)
#define PROTIFFTAG_STATFRAMESCOMPLETED     (PROTIFFTAG_BASE+21)
#define PROTIFFTAG_STATFRAMESDROPPED       (PROTIFFTAG_BASE+22)
#define PROTIFFTAG_STREAMBYTESPERSECOND    (PROTIFFTAG_BASE+23)
#define PROTIFFTAG_WIDTH                   (PROTIFFTAG_BASE+24)

// image info
#define PROTIFFTAG_TIME_LO                 (PROTIFFTAG_BASE+30)
#define PROTIFFTAG_TIME_HI                 (PROTIFFTAG_BASE+31)
#define PROTIFFTAG_IMG_NUM                 (PROTIFFTAG_BASE+32)

static const TIFFFieldInfo ProTiffFieldInfo[]={
        { PROTIFFTAG_MAGIC,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"MagicNumber" },
        { PROTIFFTAG_VERSION,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"Version" },
        { PROTIFFTAG_BINNINGX,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"BinningX" },
        { PROTIFFTAG_BINNINGY,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"BinningY" },
        { PROTIFFTAG_EXPOSUREAUTOTARGET,  1, 1,	TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"ExposureAutoTarget" },
        { PROTIFFTAG_EXPOSUREMODE,	-1, -1,		TIFF_ASCII,	FIELD_CUSTOM,
                1,	0,	"ExposureMode" },
        { PROTIFFTAG_EXPOSUREVALUE,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"ExposureValue" },
        { PROTIFFTAG_FRAMERATE,	1, 1,		TIFF_FLOAT,	FIELD_CUSTOM,
                1,	0,	"FrameRate" },
        { PROTIFFTAG_GAINVALUE,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"GainValue" },
        { PROTIFFTAG_HEIGHT,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"Height" },
        { PROTIFFTAG_REGIONX,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"RegionX" },
        { PROTIFFTAG_REGIONY,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"RegionY" },
        { PROTIFFTAG_STATFRAMERATE, 1, 1,		TIFF_FLOAT,	FIELD_CUSTOM,
                1,	0,	"StatFrameRate" },
        { PROTIFFTAG_STATFRAMESCOMPLETED,   1, 1,	    TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"StatFramesCompleted" },
        { PROTIFFTAG_STATFRAMESDROPPED,    1, 1,	   TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"StatFramesDropped" },
        { PROTIFFTAG_STREAMBYTESPERSECOND, 1, 1,	    TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"StreamBytesPerSecond" },
        { PROTIFFTAG_WIDTH,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"Width" },
        { PROTIFFTAG_TIME_LO,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"TimestampLo" },
        { PROTIFFTAG_TIME_HI,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"TimestampHi" },
        { PROTIFFTAG_IMG_NUM,	1, 1,		TIFF_LONG,	FIELD_CUSTOM,
                1,	0,	"ImgNum" }
};

#define PROTIFFTAG_N (sizeof(ProTiffFieldInfo)/sizeof(ProTiffFieldInfo[0]))

#endif //TIFTAGS_H
