/************************************************************************

*  Copyright (C) Tucsen Photonics Co.,Ltd. 2012-2025. All rights reserved.

*  @file      TUDefine.h

*  @brief     The Tucsen API defines

*  @version	  1.0.0.0	    

*  @author    Zhang Ren

*  @date      2023-01-05

************************************************************************/
#ifndef _TUDEFINE_H_
#define _TUDEFINE_H_

#ifndef TUCAMAPI_VER
#define	TUCAMAPI_VER	1000            // TUCAM-API Version
#endif


#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#ifdef _WIN64
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif
/* **************************************************************** *

    platform absorber

 * **************************************************************** */

//  determine TARGET OS
#if !defined( TUCAM_TARGETOS_IS_WIN32 ) && !defined( TUCAM_TARGETOS_IS_MACOSX ) && !defined( TUCAM_TARGETOS_IS_LINUX )

#if defined( WIN32 ) || defined( _WIN64 ) || defined( _INC_WINDOWS )

#ifndef _INC_WINDOWS
#error WINDOWS.H is not included.
#endif  // _INC_WINDOWS

#define	TUCAM_TARGETOS_IS_WIN32
#define	_PHX_WIN

#elif defined( LINUX )

#define	TUCAM_TARGETOS_IS_LINUX

#elif defined( MACOSX ) || __ppc64__ || __i386__ || __x86_64__

#define	TUCAM_TARGETOS_IS_MACOSX

#else

//  now TUCAM only supports Windows, Linux and MacOSX

#error TUCAM requires one of definition WIN32, MACOSX and LINUX

#endif  // #if defined( WIN32 ) || defined( _INC_WINDOWS )

#endif	// #if ! defined( TUCAM_TARGETOS_IS_WIN32 ) && ! defined( TUCAM_TARGETOS_IS_MACOSX ) && ! defined( TUCAM_TARGETOS_IS_LINUX )


/* **************************************************************** *

    defines

* **************************************************************** */

typedef struct _tagTUCAM*  HDTUCAM;      // TUCAM hander pointer
typedef struct _tagTUFRM*  HDTUIMG;      // TUIMG hander pointer

#ifdef TUCAM_TARGETOS_IS_WIN32          // define for win32

// ms
#ifndef TUCAM_SLEEP
#define TUCAM_SLEEP(x)      Sleep(x)
#endif

typedef int                 INT;
typedef unsigned int        UINT;

typedef short               SHORT;
typedef unsigned short      USHORT;

typedef char                CHAR;
typedef char*               PCHAR;
typedef unsigned char       UCHAR;
typedef unsigned char*      PUCHAR;

typedef double              DOUBLE;

typedef long                int32_;
typedef	unsigned long       uint32_;
typedef	unsigned long       DWORD_;

#else

typedef int                 int32_;
typedef unsigned int        uint32_;
typedef	unsigned int        DWORD_;
typedef unsigned char       UCHAR;

#endif  // TUCAM_TARGETOS_IS_WIN32

#ifdef TUCAM_TARGETOS_IS_LINUX          // define for Linux

#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h> 
#include <cstring>

// ms
#ifndef TUCAM_SLEEP
#define TUCAM_SLEEP(x)      {x >= 1000 ? sleep(x/1000) : usleep(x*1000);}
#endif

#ifndef CALLBACK
#define CALLBACK            __attribute__(())
#endif

#ifndef INFINITE
#define INFINITE            0xFFFFFFFF
#endif

typedef unsigned int        BOOL;

typedef int                 INT;
typedef int*                PINT;
typedef uint32_t            UINT;
typedef uint16_t            UINT16;
typedef int32_t             INT32;
typedef int32_t*            PINT32;
typedef uint32_t            UINT32;
typedef uint32_t*           PUINT32;
typedef uint64_t            UINT64;
typedef void*               PVOID;
typedef void*               LPVOID;
typedef long long           INT64;

typedef char                CHAR;
typedef char*               PCHAR;
typedef unsigned char       UCHAR;
typedef unsigned char*      PUCHAR;

typedef short               SHORT;
typedef short*              PSHORT;
typedef unsigned short      USHORT;
typedef unsigned short      WORD;
typedef unsigned short*     PUSHORT;

typedef unsigned char       BYTE;
typedef long                LONG;
typedef float               FLOAT;
typedef double              DOUBLE;
typedef unsigned long       DWORD;

#undef NULL
#if defined(__cplusplus)
#define NULL 0
#else
#define NULL ((void *)0)
#endif

#endif  // TUCAM_TARGETOS_IS_LINUX

#ifdef TUCAM_TARGETOS_IS_MACOSX         // define for MacOSX

#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include <cstring>

// ms
#ifndef TUCAM_SLEEP
#define TUCAM_SLEEP(x)      {x >= 1000 ? sleep(x/1000) : usleep(x*1000);}
#endif

#ifndef INFINITE
#define INFINITE            0xFFFFFFFF
#endif

#ifndef CALLBACK
#define CALLBACK            __attribute__(())
#endif

typedef int                 INT;
typedef int*                PINT;
typedef uint32_             UINT;
typedef int32_              INT32;
typedef uint16_t            UINT16;
typedef int32_*             PINT32;
typedef uint32_             UINT32;
typedef uint32_*            PUINT32;
typedef unsigned long long  UINT64;
typedef void*               PVOID;
typedef void*               LPVOID;
typedef long long           INT64;

typedef char                CHAR;
typedef char*               PCHAR;
typedef unsigned char       UCHAR;
typedef unsigned char*      PUCHAR;

typedef short               SHORT;
typedef short*              PSHORT;
typedef unsigned short      USHORT;
typedef unsigned short*     PUSHORT;

typedef unsigned char       BYTE;
typedef long                LONG;
typedef float               FLOAT;
typedef double              DOUBLE;
typedef unsigned long       DWORD;

#ifndef _OBJC_OBJC_H_
typedef signed char         BOOL;
#endif

#endif  // TUCAM_TARGETOS_IS_MACOSX

#ifndef TUCAM_UNREFERENCED_PARAMETER
#define TUCAM_UNREFERENCED_PARAMETER(p) (void)(p)
#endif

#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

#ifndef TUCAM_DELOBJ
#define TUCAM_DELOBJ(pObj)  {if(NULL != pObj) delete pObj; pObj = NULL;}
#endif

#ifndef TUCAM_DELBUF
#define TUCAM_DELBUF(pBuf)  {if(NULL != pBuf) delete [] pBuf; pBuf = NULL;}
#endif

#ifndef TUCAM_FREEBUF
#define TUCAM_FREEBUF(pBuf) {if(NULL != pBuf) free(pBuf); pBuf = NULL;}
#endif

#define TUCAM_TIMEOUT       1000 // TimeOut 1000ms

#define TUSN_SIZE           64
#define TUCODE_SIZE         128

#define TUSIZE_SN           TUSN_SIZE
#define TUSIZE_DPC          2001
#define TUSIZE_CODE         TUCODE_SIZE

/* **************************************************************** *

    enum defines

* **************************************************************** */

//  The correct state [0x00000000, 0x7FFFFFFF] 
//  The error   state [0x80000000, 0XFFFFFFFF]
//  typedef enum TUCAM status
typedef enum
{
    //  success
    TUCAMRET_SUCCESS            = 0x00000001,       // no error, general success code, app should check the value is positive   
    TUCAMRET_FAILURE            = 0x80000000,       // error    

    //  initialization error
    TUCAMRET_NO_MEMORY          = 0x80000101,       // not enough memory
    TUCAMRET_NO_RESOURCE        = 0x80000102,       // not enough resource except memory    
    TUCAMRET_NO_MODULE          = 0x80000103,       // no sub module
    TUCAMRET_NO_DRIVER          = 0x80000104,       // no driver
    TUCAMRET_NO_CAMERA          = 0x80000105,       // no camera
    TUCAMRET_NO_GRABBER         = 0x80000106,       // no grabber  
    TUCAMRET_NO_PROPERTY        = 0x80000107,       // there is no alternative or influence id, or no more property id

    TUCAMRET_FAILOPEN_CAMERA    = 0x80000110,       // fail open the camera
    TUCAMRET_FAILOPEN_BULKIN    = 0x80000111,       // fail open the bulk in endpoint
    TUCAMRET_FAILOPEN_BULKOUT   = 0x80000112,       // fail open the bulk out endpoint
    TUCAMRET_FAILOPEN_CONTROL   = 0x80000113,       // fail open the control endpoint
    TUCAMRET_FAILCLOSE_CAMERA   = 0x80000114,       // fail close the camera

    TUCAMRET_FAILOPEN_FILE      = 0x80000115,       // fail open the file
	TUCAMRET_FAILOPEN_CODEC		= 0x80000116,       // fail open the video codec
	TUCAMRET_FAILOPEN_CONTEXT   = 0x80000117,       // fail open the video context

    //  status error
    TUCAMRET_INIT               = 0x80000201,       // API requires has not initialized state.
    TUCAMRET_BUSY               = 0x80000202,       // API cannot process in busy state.
    TUCAMRET_NOT_INIT           = 0x80000203,       // API requires has initialized state.
    TUCAMRET_EXCLUDED           = 0x80000204,       // some resource is exclusive and already used.
    TUCAMRET_NOT_BUSY           = 0x80000205,       // API requires busy state.
    TUCAMRET_NOT_READY          = 0x80000206,       // API requires ready state.
    
    //  wait error
    TUCAMRET_ABORT              = 0x80000207,       // abort process
    TUCAMRET_TIMEOUT            = 0x80000208,       // timeout
    TUCAMRET_LOSTFRAME          = 0x80000209,       // frame data is lost
    TUCAMRET_MISSFRAME          = 0x8000020A,       // frame is lost but reason is low lever driver's bug
    TUCAMRET_USB_STATUS_ERROR   = 0x8000020B,       // the USB status error
    TUCAMRET_FRAME_BUFFER_FULL  = 0x8000020C,       // the frame ring buffer is full

    // calling error
    TUCAMRET_INVALID_CAMERA     = 0x80000301,       // invalid camera
    TUCAMRET_INVALID_HANDLE     = 0x80000302,       // invalid camera handle
    TUCAMRET_INVALID_OPTION     = 0x80000303,       // invalid the option value of structure
    TUCAMRET_INVALID_IDPROP     = 0x80000304,       // invalid property id
    TUCAMRET_INVALID_IDCAPA     = 0x80000305,       // invalid capability id
    TUCAMRET_INVALID_IDPARAM    = 0x80000306,       // invalid parameter id
    TUCAMRET_INVALID_PARAM      = 0x80000307,       // invalid parameter
    TUCAMRET_INVALID_FRAMEIDX   = 0x80000308,       // invalid frame index
    TUCAMRET_INVALID_VALUE      = 0x80000309,       // invalid property value
    TUCAMRET_INVALID_EQUAL      = 0x8000030A,       // invalid property value equal 
    TUCAMRET_INVALID_CHANNEL    = 0x8000030B,       // the property id specifies channel but channel is invalid
    TUCAMRET_INVALID_SUBARRAY   = 0x8000030C,       // the combination of subarray values are invalid. e.g. TUCAM_IDPROP_SUBARRAYHPOS + TUCAM_IDPROP_SUBARRAYHSIZE is greater than the number of horizontal pixel of sensor.
    TUCAMRET_INVALID_VIEW       = 0x8000030D,       // invalid view window handle
    TUCAMRET_INVALID_PATH       = 0x8000030E,       // invalid file path
    TUCAMRET_INVALID_IDVPROP    = 0x8000030F,       // invalid vendor property id

    TUCAMRET_NO_VALUETEXT       = 0x80000310,       // the property does not have value text
    TUCAMRET_OUT_OF_RANGE       = 0x80000311,       // value is out of range

    TUCAMRET_NOT_SUPPORT        = 0x80000312,       // camera does not support the function or property with current settings
    TUCAMRET_NOT_WRITABLE       = 0x80000313,       // the property is not writable	
    TUCAMRET_NOT_READABLE       = 0x80000314,       // the property is not readable

             
    TUCAMRET_WRONG_HANDSHAKE    = 0x80000410,       // this error happens TUCAM get error code from camera unexpectedly
    TUCAMRET_NEWAPI_REQUIRED    = 0x80000411,       // old API does not support the value because only new API supports the value
 
    TUCAMRET_ACCESSDENY         = 0x80000412,       // the property cannot access during this TUCAM status

    TUCAMRET_NO_CORRECTIONDATA  = 0x80000501,       // not take the dark and shading correction data yet.

    TUCAMRET_INVALID_PRFSETS    = 0x80000601,       // the profiles set name is invalid
    TUCAMRET_INVALID_IDPPROP    = 0x80000602,       // invalid process property id

    TUCAMRET_DECODE_FAILURE     = 0x80000701,       // the image decoding raw data to rgb data failure
    TUCAMRET_COPYDATA_FAILURE   = 0x80000702,       // the image data copying failure
	TUCAMRET_ENCODE_FAILURE		= 0x80000703,		// the image encoding data  to video failure
	TUCAMRET_WRITE_FAILURE		= 0x80000704,		// the write the video frame failure

    //  camera or bus trouble
    TUCAMRET_FAIL_READ_CAMERA   = 0x83001001,       // fail read from camera  
    TUCAMRET_FAIL_WRITE_CAMERA  = 0x83001002,       // fail write to camera
    TUCAMRET_OPTICS_UNPLUGGED   = 0x83001003,       // optics part is unplugged so please check it.

	TUCAMRET_RECEIVE_FINISH     = 0x00000002,       // no error, vendor receive frame message  
    TUCAMRET_EXTERNAL_TRIGGER   = 0x00000003,       // no error, receive the external trigger signal

}TUCAMRET;

//  typedef enum information id
typedef enum
{
    TUIDI_BUS                   = 0x01,             // the bus type USB2.0/USB3.0
    TUIDI_VENDOR                = 0x02,             // the vendor id
    TUIDI_PRODUCT               = 0x03,             // the product id 
    TUIDI_VERSION_API           = 0x04,             // the API version    
    TUIDI_VERSION_FRMW          = 0x05,             // the firmware version
    TUIDI_VERSION_FPGA          = 0x06,             // the FPGA version
    TUIDI_VERSION_DRIVER        = 0x07,             // the driver version
    TUIDI_TRANSFER_RATE         = 0x08,             // the transfer rate
    TUIDI_CAMERA_MODEL          = 0x09,             // the camera model (string)
    TUIDI_CURRENT_WIDTH         = 0x0A,             // the camera image data current width(must use TUCAM_Dev_GetInfoEx and after calling TUCAM_Buf_Alloc)
    TUIDI_CURRENT_HEIGHT        = 0x0B,             // the camera image data current height(must use TUCAM_Dev_GetInfoEx and after calling TUCAM_Buf_Alloc)
    TUIDI_CAMERA_CHANNELS       = 0x0C,             // the camera image data channels
    TUIDI_BCDDEVICE             = 0x0D,             // the USB bcdDevice
	TUIDI_TEMPALARMFLAG         = 0x0E,             // the Temperature Alarm Flag
	TUIDI_UTCTIME               = 0x0F,             // the get utc time
	TUIDI_LONGITUDE_LATITUDE    = 0x10,             // the get longitude latitude
	TUIDI_WORKING_TIME          = 0x11,             // the get working time
	TUIDI_FAN_SPEED             = 0x12,             // the get fan speed
	TUIDI_FPGA_TEMPERATURE      = 0x13,             // the get fpga temperature
	TUIDI_PCBA_TEMPERATURE      = 0x14,             // the get pcba temperature
	TUIDI_ENV_TEMPERATURE       = 0x15,             // the get environment temperature
    TUIDI_DEVICE_ADDRESS        = 0x16,             // the USB device address
    TUIDI_USB_PORT_ID           = 0x17,             // the USB port id
	TUIDI_CONNECTSTATUS         = 0x18,             // the camera whether connection
	TUIDI_TOTALBUFFRAMES        = 0x19,             // the USB total buffer frames
	TUIDI_CURRENTBUFFRAMES      = 0x1A,             // the USB current buffer frames
	TUIDI_HDRRATIO              = 0x1B,             // the get hdr ratio value
	TUIDI_HDRKHVALUE            = 0x1C,             // the get hdr kh value
	TUIDI_ZEROTEMPERATURE_VALUE = 0x1D,             // the get zero temperature reference value
	TUIDI_VALID_FRAMEBIT        = 0x1E,             // the get valid frame bit
	TUIDI_CONFIG_HDR_HIGH_GAIN_K,
	TUIDI_CONFIG_HDR_RATIO,
	TUIDI_ENDINFO,             // the string id end
}TUCAM_IDINFO;

// typedef enum capability id 
typedef enum
{
    TUIDC_RESOLUTION            = 0x00,             // id capability resolution
    TUIDC_PIXELCLOCK            = 0x01,             // id capability pixel clock
    TUIDC_BITOFDEPTH            = 0x02,             // id capability bit of depth
    TUIDC_ATEXPOSURE            = 0x03,             // id capability automatic exposure time  
    TUIDC_HORIZONTAL            = 0x04,             // id capability horizontal
    TUIDC_VERTICAL              = 0x05,             // id capability vertical
    TUIDC_ATWBALANCE            = 0x06,             // id capability automatic white balance
    TUIDC_FAN_GEAR              = 0x07,             // id capability fan gear
    TUIDC_ATLEVELS              = 0x08,             // id capability automatic levels
    TUIDC_SHIFT                 = 0x09,             // (The reserved) id capability shift(15~8, 14~7, 13~6, 12~5, 11~4, 10~3, 9~2, 8~1, 7~0) [16bit]
    TUIDC_HISTC                 = 0x0A,             // id capability histogram statistic
    TUIDC_CHANNELS              = 0x0B,             // id capability current channels(Only color camera support:0-RGB,1-Red,2-Green,3-Blue. Used in the property levels, see enum TUCHN_SELECT)
    TUIDC_ENHANCE               = 0x0C,             // id capability enhance
    TUIDC_DFTCORRECTION         = 0x0D,             // id capability defect correction (0-not correction, 1-calculate, 3-correction)
    TUIDC_ENABLEDENOISE         = 0x0E,             // id capability enable denoise (TUIDP_NOISELEVEL effective)
    TUIDC_FLTCORRECTION         = 0x0F,             // id capability flat field correction (0-not correction, 1-grab frame, 2-calculate, 3-correction)
    TUIDC_RESTARTLONGTM         = 0x10,             // id capability restart long exposure time (only CCD camera support)
    TUIDC_DATAFORMAT            = 0x11,             // id capability the data format(only YUV format data support 0-YUV 1-RAW)
    TUIDC_DRCORRECTION          = 0x12,             // (The reserved)id capability dynamic range of correction
    TUIDC_VERCORRECTION         = 0x13,             // id capability vertical correction(correction the image data show vertical, in windows os the default value is 1)
    TUIDC_MONOCHROME            = 0x14,             // id capability monochromatic
    TUIDC_BLACKBALANCE          = 0x15,             // id capability black balance
    TUIDC_IMGMODESELECT         = 0x16,             // id capability image mode select(CMS mode)
    TUIDC_CAM_MULTIPLE          = 0x17,             // id capability multiple cameras (how many cameras use at the same time, only SCMOS camera support)
	TUIDC_ENABLEPOWEEFREQUENCY  = 0x18,             // id capability enable power frequency (50HZ or 60HZ)
	TUIDC_ROTATE_R90			= 0x19,				// id capability rotate 90 degree to right
	TUIDC_ROTATE_L90			= 0x1A,				// id capability rotate 90 degree to left
	TUIDC_NEGATIVE				= 0x1B,				// id capability negative film enable
	TUIDC_HDR					= 0x1C,				// id capability HDR enable
    TUIDC_ENABLEIMGPRO          = 0x1D,             // id capability image process enable
    TUIDC_ENABLELED             = 0x1E,             // id capability USB led enable
    TUIDC_ENABLETIMESTAMP       = 0x1F,             // id capability time stamp enable
    TUIDC_ENABLEBLACKLEVEL      = 0x20,				// id capability black level offset enable
    TUIDC_ATFOCUS               = 0x21,             // id capability auto focus enable(0-manual 1-automatic focus 2-Once)
    TUIDC_ATFOCUS_STATUS        = 0x22,             // id capability auto focus status(0-stop 1-focusing 2-completed 3-defocus)
	TUIDC_PGAGAIN               = 0x23,             // id capability sensor pga gain
    TUIDC_ATEXPOSURE_MODE       = 0x24,             // id capability automatic exposure time mode
    TUIDC_BINNING_SUM           = 0x25,             // id capability the summation binning
    TUIDC_BINNING_AVG           = 0x26,             // id capability the average binning
    TUIDC_FOCUS_C_MOUNT         = 0x27,             // id capability the focus c-mount mode(0-normal 1-c-mount mode)
	TUIDC_ENABLEPI              = 0x28,             // id capability PI enable
    TUIDC_ATEXPOSURE_STATUS     = 0x29,             // id capability auto exposure status (0-doing 1-completed)
    TUIDC_ATWBALANCE_STATUS     = 0x2A,             // id capability auto white balance status (0-doing 1-completed)
	TUIDC_TESTIMGMODE           = 0x2B,             // id capability test image mode select
	TUIDC_SENSORRESET           = 0x2C,             // id capability sensor reset
	TUIDC_PGAHIGH               = 0x2D,             // id capability pga high gain
	TUIDC_PGALOW                = 0x2E,             // id capability pga low gain
	TUIDC_PIXCLK1_EN            = 0x2F,             // id capability pix1 clock enable
	TUIDC_PIXCLK2_EN            = 0x30,             // id capability pix2 clock enable
	TUIDC_ATLEVELGEAR           = 0x31,             // id capability auto level gear
    TUIDC_ENABLEDSNU            = 0x32,             // id capability enable dsnu
    TUIDC_ENABLEOVERLAP         = 0x33,             // id capability enable exposure time overlap mode
	TUIDC_CAMSTATE              = 0x34,             // id capability camera state
	TUIDC_ENABLETRIOUT          = 0x35,             // id capability enable trigger out enable
	TUIDC_ROLLINGSCANMODE       = 0x36,             // id capability rolling scan mode
	TUIDC_ROLLINGSCANLTD        = 0x37,             // id capability rolling scan line time delay
	TUIDC_ROLLINGSCANSLIT       = 0x38,             // id capability rolling scan slit height
	TUIDC_ROLLINGSCANDIR        = 0x39,             // id capability rolling scan direction
	TUIDC_ROLLINGSCANRESET      = 0x3A,             // id capability rolling scan direction reset
	TUIDC_ENABLETEC             = 0x3B,             // id capability TEC enable
	TUIDC_ENABLEBLC             = 0x3C,             // id capability backlight compensation enable
	TUIDC_ENABLETHROUGHFOG      = 0x3D,             // id capability electronic through fog enable
	TUIDC_ENABLEGAMMA           = 0x3E,             // id capability gamma enable
	TUIDC_ENABLEFILTER          = 0x3F,             // id capability filter enable
	TUIDC_ENABLEHLC             = 0x40,             // id capability strong light inhibition enable
	TUIDC_CAMPARASAVE           = 0x41,             // id capability camera parameter save
	TUIDC_CAMPARALOAD           = 0x42,             // id capability camera parameter load
    TUIDC_ENABLEISP             = 0x43,             // id capability camera isp enable 
	TUIDC_BUFFERHEIGHT          = 0x44,             // id capability buffer height
	TUIDC_VISIBILITY            = 0x45,             // id capability visibility
    TUIDC_SHUTTER               = 0x46,             // id capability shutter mode 
	TUIDC_SIGNALFILTER          = 0x47,             // id capability signal filter
    TUIDC_ATEXPOSURE_TYPE       = 0x48,             // id capability automatic exposure time type(0-Auto(Exposure And Gain) 1-Auto(Only Exposure) 2-Auto(Only Gain))
    TUIDC_ENDCAPABILITY         = 0x49,             // id capability end 
}TUCAM_IDCAPA;

// Fix 
#define TUIDC_IMGPROENBALE    TUIDC_ENABLEIMGPRO    // id capability image process enable
#define TUIDC_LEDENBALE       TUIDC_ENABLELED       // id capability USB led enable
#define TUIDC_TIMESTAMPENBALE TUIDC_ENABLETIMESTAMP // id capability time stamp enable

// typedef enum property id
typedef enum
{   
    TUIDP_GLOBALGAIN            = 0x00,             // id property global gain
    TUIDP_EXPOSURETM            = 0x01,             // id property exposure time
    TUIDP_BRIGHTNESS            = 0x02,             // id property brightness (Effective automatic exposure condition)
    TUIDP_BLACKLEVEL            = 0x03,             // id property black level
    TUIDP_TEMPERATURE           = 0x04,             // id property temperature control
    TUIDP_SHARPNESS             = 0x05,             // id property sharpness
    TUIDP_NOISELEVEL            = 0x06,             // id property the noise level
    TUIDP_HDR_KVALUE            = 0x07,             // id property the HDR K value

    // image process property
    TUIDP_GAMMA                 = 0x08,             // id property gamma
    TUIDP_CONTRAST              = 0x09,             // id property contrast
    TUIDP_LFTLEVELS             = 0x0A,             // id property left levels
    TUIDP_RGTLEVELS             = 0x0B,             // id property right levels
    TUIDP_CHNLGAIN              = 0x0C,             // id property channel gain
    TUIDP_SATURATION            = 0x0D,             // id property saturation
    TUIDP_CLRTEMPERATURE        = 0x0E,             // id property color temperature
    TUIDP_CLRMATRIX             = 0x0F,             // id property color matrix setting
    TUIDP_DPCLEVEL              = 0x10,             // id property defect points correction level
    TUIDP_BLACKLEVELHG          = 0x11,             // id property black level high gain
    TUIDP_BLACKLEVELLG          = 0x12,             // id property black level low gain
	TUIDP_POWEEFREQUENCY        = 0x13,             // id property power frequency (50HZ or 60HZ)
	TUIDP_HUE					= 0x14,				// id property hue
	TUIDP_LIGHT					= 0x15,				// id property light
    TUIDP_ENHANCE_STRENGTH  	= 0x16,				// id property enhance strength
    TUIDP_NOISELEVEL_3D         = 0x17,				// id property the 3D noise level
    TUIDP_FOCUS_POSITION        = 0x18,             // id property focus position
	
	TUIDP_FRAME_RATE            = 0x19,             // id property frame rate
	TUIDP_START_TIME            = 0x1A,             // id property start time
	TUIDP_FRAME_NUMBER          = 0x1B,             // id property frame number
	TUIDP_INTERVAL_TIME         = 0x1C,             // id property interval time
	TUIDP_GPS_APPLY             = 0x1D,             // id property gps apply
	TUIDP_AMB_TEMPERATURE       = 0x1E,             // id property ambient temperature
	TUIDP_AMB_HUMIDITY          = 0x1F,             // id property ambient humidity
	TUIDP_AUTO_CTRLTEMP         = 0x20,             // id property auto control temperature

	TUIDP_AVERAGEGRAY           = 0x21,             // id property average gray setting
	TUIDP_AVERAGEGRAYTHD        = 0x22,             // id property average gray threshold setting
	TUIDP_ENHANCETHD            = 0x23,             // id property enhance threshold setting
	TUIDP_ENHANCEPARA           = 0x24,             // id property enhance parameter setting
	TUIDP_EXPOSUREMAX           = 0x25,             // id property max exposure time setting
	TUIDP_EXPOSUREMIN           = 0x26,             // id property min exposure time setting
	TUIDP_GAINMAX               = 0x27,             // id property max gain setting
	TUIDP_GAINMIN               = 0x28,             // id property min gain setting
	TUIDP_THROUGHFOGPARA        = 0x29,             // id property through fog parameter setting
    TUIDP_ATLEVEL_PERCENTAGE    = 0x2A,             // id property auto levels ignore percentage
    TUIDP_TEMPERATURE_TARGET    = 0x2B,             // id property temperature target

	TUIDP_PIXELRATIO            = 0x2C,             // id property pixel ratio

	TUIDP_ENDPROPERTY           = 0x2D,             // id property end 
}TUCAM_IDPROP;

// typedef enum vendor property id
typedef enum
{   
    TUIDV_ADDR_FLASH            = 0x00,             // id vendor flash address
	TUIDV_ODDEVENH              = 0x01,             // id vendor odd even high value
	TUIDV_ODDEVENL              = 0x02,             // id vendor odd even low value
	TUIDV_HDRHGBOFFSET          = 0x03,             // id vendor the hdr high gain b offset
	TUIDV_HDRLGBOFFSET          = 0x04,             // id vendor the hdr low gain b offset
	TUIDV_CMSHGBOFFSET          = 0x05,             // id vendor the cms high gain b offset
	TUIDV_CMSLGBOFFSET          = 0x06,             // id vendor the cms low gain b offset
	TUIDV_FPNENABLE             = 0x07,             // id vendor the fpn enable
    TUIDV_WORKING_TIME          = 0x08,             // id vendor the working time
    TUIDV_CALC_DSNU             = 0x09,             // id vendor the calc dsnu
    TUIDV_CALC_PRNU             = 0x0A,             // id vendor the calc prnu
    TUIDV_CALC_DPC              = 0x0B,             // id vendor the calc dpc
    TUIDV_CALC_STOP             = 0x0C,             // id vendor the calc stop
    TUIDV_CALC_STATE            = 0x0D,             // id vendor the calc state [0-GenFree, 1-GenBusy, 2-CalBusy, 3-WRBusy, 4-GenDone, 5-GenStop]
    TUIDV_HDR_LVALUE            = 0x0E,             // id property the HDR Low value
    TUIDV_HDR_HVALUE            = 0x0F,             // id property the HDR High value
	TUIDV_FW_CHECK              = 0x10,             // id property the FW Check value
    TUIDV_HIGHSPEEDHGBOFFSET    = 0x11,             // id vendor the high speed high gain b offset
	TUIDV_HIGHSPEEDLGBOFFSET    = 0x12,             // id vendor the high speed low gain b offset
	TUIDV_TEMPERATURE_OFFSET    = 0x13,             // id vendor the temperature offset
	TUIDV_DIGITALHGCOE          = 0x14,             // id vendor the digital hg coe
	TUIDV_DIGITALLGCOE          = 0x15,             // id vendor the digital lg coe
	TUIDV_DIGITALKCOE           = 0x16,             // id vendor the digital k coe
	TUIDV_BLACKLEVELHG          = 0x17,             // id vendor the black level high gain
	TUIDV_BLACKLEVELLG          = 0x18,             // id vendor the black level low gain
	TUIDV_HDR_KVALUE            = 0x19,             // id vendor the HDR K value
	TUIDV_BACKGROUND_OFFSET = 0x1A,             // id vendor the Background offset
	TUIDV_MAX_FRAME_RATE = 0x1B,             // id vendor the max frame rate
    TUIDV_ENDVPROPERTY = 0x1C,             // id vendor end 
}TUCAM_IDVPROP;

// typedef enum calculate roi id
typedef enum
{
    TUIDPP_EDF_QUALITY          = 0x00,             // id process EDF quality
    TUIDPP_STITCH_SPEED         = 0x01,             // id process stitch speed
    TUIDPP_STITCH_BGC_RED       = 0x02,             // id process stitch background color red
    TUIDPP_STITCH_BGC_GREEN     = 0x03,             // id process stitch background color green
    TUIDPP_STITCH_BGC_BLUE      = 0x04,             // id process stitch background color blue
    TUIDPP_STITCH_VALID         = 0x05,             // id process stitch whether the result is valid (Only get value)
    TUIDPP_STITCH_AREA_X        = 0x06,             // id process stitch result and the current point X coordinates value (Only get value)
    TUIDPP_STITCH_AREA_Y        = 0x07,             // id process stitch result and the current point Y coordinates value (Only get value)
    TUIDPP_STITCH_NEXT_X        = 0x08,             // id process stitch result and the next point X coordinates value (Only get value)
    TUIDPP_STITCH_NEXT_Y        = 0x09,             // id process stitch result and the next point Y coordinates value (Only get value)
    TUIDPP_ENDPPROPERTY         = 0x0A,             // id process end 
}TUCAM_IDPPROP;

// typedef enum calculate roi id
typedef enum
{   
    TUIDCR_WBALANCE             = 0x00,             // id calculate roi white balance
    TUIDCR_BBALANCE             = 0x01,             // id calculate roi black balance
    TUIDCR_BLOFFSET             = 0x02,             // id calculate roi black level offset
    TUIDCR_FOCUS                = 0x03,             // id calculate roi focus
    TUIDCR_EXPOSURETM           = 0x04,             // id calculate roi exposure time
    TUIDCR_END                  = 0x05,             // id calculate roi end
}TUCAM_IDCROI;

// typedef enum the capture mode
typedef enum
{
    TUCCM_SEQUENCE              = 0x00,             // capture start sequence mode
    TUCCM_TRIGGER_STANDARD      = 0x01,             // capture start trigger standard mode
    TUCCM_TRIGGER_SYNCHRONOUS   = 0x02,             // capture start trigger synchronous mode
    TUCCM_TRIGGER_GLOBAL        = 0x03,             // capture start trigger global
    TUCCM_TRIGGER_SOFTWARE      = 0x04,             // capture start trigger software
	TUCCM_TRIGGER_GPS           = 0x05,             // capture start trigger gps
	TUCCM_TRIGGER_STANDARD_NONOVERLAP = 0x11,       // capture start trigger standard mode(non-overlap)
}TUCAM_CAPTURE_MODES;

// typedef enum the image formats
typedef enum 
{
    TUFMT_RAW                   = 0x01,             // The format RAW
    TUFMT_TIF                   = 0x02,             // The format TIFF
    TUFMT_PNG                   = 0x04,             // The format PNG
    TUFMT_JPG                   = 0x08,             // The format JPEG
    TUFMT_BMP                   = 0x10,             // The format BMP
}TUIMG_FORMATS;

// typedef enum the register types
typedef enum 
{
    TUREG_SN                    = 0x01,             // The type register SN
    TUREG_DATA                  = 0x02,             // The type register data
    TUREG_BAD_ROW               = 0x03,             // The type register bad row     (Vendor use)
    TUREG_BAD_COL               = 0x04,             // The type register bad column  (Vendor use)
    TUREG_BGC                   = 0x05,             // The type register background  (Vendor use)
    TUREG_HDR                   = 0x06,             // The type register HDR exp para(Vendor use)
    TUREG_HBG                   = 0x07,             // The type register HDR exp para(Vendor use)
    TUREG_CMS                   = 0x08,             // The type register CMS exp para(Vendor use)
    TUREG_CBG                   = 0x09,             // The type register CMS exp para(Vendor use)
    TUREG_CODE                  = 0x0A,             // The type register code        (Vendor use)
    TUREG_DPC                   = 0x0B,             // The type register DPC         (Vendor use)
	TUREG_TEMPERATUREOFFSET     = 0x0C,             // The type register Temperature (Vendor use)
	TUREG_HIGHSPEEDBGYLIST      = 0x0D,             // The type register  (Vendor use)
 }TUREG_TYPE;

// trigger mode
typedef enum
{
    TUCTS_TIMED = 0x00,             // timed mode
    TUCTD_WIDTH_START = 0x01,       // width start
    TUCTD_WIDTH_STOP = 0x02,        // width stop
}TUCAM_TRIGGER_SOFTWARE;

// typedef enum the trigger exposure time mode
typedef enum 
{
    TUCTE_EXPTM                 = 0x00,             // use exposure time 
    TUCTE_WIDTH                 = 0x01,             // use width level
}TUCAM_TRIGGER_EXP;

// typedef enum the trigger edge mode
typedef enum 
{
    TUCTD_RISING                = 0x01,             // rising edge
    TUCTD_FAILING               = 0x00,             // failing edge
}TUCAM_TRIGGER_EDGE;

// typedef enum the trigger readout direction reset mode
typedef enum
{
	TUCTD_YES                   = 0x00,            // yes reset
	TUCTD_NO                    = 0x01,            // no reset
}TUCAM_TRIGGER_READOUTDIRRESET;

// typedef enum the trigger readout direction mode
typedef enum
{
	TUCTD_DOWN                  = 0x00,            // down
	TUCTD_UP                    = 0x01,            // up
	TUCTD_DOWNUPCYC             = 0x02,            // down up cycle
}TUCAM_TRIGGER_READOUTDIR;


// outputtrigger mode
// typedef enum the output trigger port mode
typedef enum 
{
	TUPORT_ONE                  = 0x00,            // use port1
	TUPORT_TWO                  = 0x01,            // use port2
	TUPORT_THREE                = 0x02,            // use port3
}TUCAM_OUTPUTTRG_PORT;

// typedef enum the output trigger kind mode
typedef enum 
{
	TUOPT_GND                   = 0x00,              // use low 
	TUOPT_VCC                   = 0x01,              // use high
	TUOPT_IN                    = 0x02,              // use trigger input
	TUOPT_EXPSTART              = 0x03,              // use exposure start
	TUOPT_EXPGLOBAL             = 0x04,              // use global exposure 
	TUOPT_READEND               = 0x05,              // use read end
	TUOPT_TRIREADY              = 0x06,              // use trigger ready
}TUCAM_OUTPUTTRG_KIND;

// typedef enum the output trigger edge mode
typedef enum 
{
	TUOPT_RISING                = 0x00,             // rising edge
	TUOPT_FAILING               = 0x01,             // failing edge
}TUCAM_OUTPUTTRG_EDGE;

// typedef enum the frame formats
typedef enum 
{
    TUFRM_FMT_RAW               = 0x10,             // The raw data
    TUFRM_FMT_USUAl             = 0x11,             // The usually data
    TUFRM_FMT_RGB888            = 0x12,             // The RGB888 data for drawing
}TUFRM_FORMATS;

// typedef enum the SCMOS gain mode
typedef enum
{
    TUGAIN_HDR                  = 0x00,             // The HDR mode
    TUGAIN_HIGH                 = 0x01,             // The High gain mode
    TUGAIN_LOW                  = 0x02,             // The Low gain mode
}TUGAIN_MODE;

// typedef enum the vendor config mode
typedef enum
{
    TUVCM_BGC                   = 0x00,             // The background correction
    TUVCM_CODE                  = 0x01,             // The code
    TUVCM_REBG                  = 0x02,             // The refresh background
    TUVCM_SN_CHECKING           = 0x03,             // The SN checking
	TUVCM_REFW                  = 0x04,             // The refresh firmware
}TUVEN_CFG_MODE;

// typedef enum the vendor configex mode
typedef enum
{
	TUVCMEX_FWTOOL              = 0x00,             // The called by fwtool
	TUVCMEX_VENDOR              = 0x01,             // The called by factory test software
	TUVCMEX_END                 = 0x02,             // The end
}TUVEN_CFGEX_MODE;

// typedef enum drawing mode(only support on windows os)
typedef enum
{
    TUDRAW_DFT                  = 0x00,             // The default mode
    TUDRAW_DIB                  = 0x01,             // The DIB mode
    TUDRAW_DX9                  = 0x02,             // The DirectX 9.0
}TUDRAW_MODE;

// The enum of channels
typedef enum
{
    TUCHN_SHARE                 = 0x00,             // The channel shared (Gray or RGB)
    TUCHN_RED                   = 0x01,             // The channel 1 (Red channel)
    TUCHN_GREEN                 = 0x02,             // The channel 2 (Green channel)
    TUCHN_BLUE                  = 0x03,             // The channel 3 (Blue channel)
}TUCHN_SELECT;

// typedef enum the firmware types
typedef enum 
{
    TUFW_IIC                    = 0x01,             // The type firmware IIC
    TUFW_FPGA                   = 0x02,             // The type firmware FPGA
	TUFW_DATA                   = 0x03,             // The type firmware DATA
}TUFW_TYPE;

// typedef enum the record append mode
typedef enum
{
    TUREC_TIMESTAMP             = 0x01,             // The record mode time-stamp 
    TUREC_SEQUENCE              = 0x02,             // The record mode sequence
}TUREC_MODE;

// typedef enum the image process type
typedef enum
{
    TUPROC_EDF                  = 0x00,             // The process EDF
    TUPROC_STITCH               = 0x01,             // The process stitch
}TUPROC_TYPE;

// typedef enum the image process stitch mode
typedef enum
{
    TUSM_FINE                   = 0x00,             // The fine mode 
    TUSM_EXCELLENT              = 0x01,             // The excellent mode
}TUSTITCH_MODE;

// typedef enum the any bin mode
typedef enum
{
	TUBIN_MODE_SUM             = 0x00,              // [in] the sum bin mode
	TUBIN_MODE_AVERAGE         = 0x01,              // [in] the average bin mode
} TUCAM_BINMODE;


// typedef enum the focus status
typedef enum
{
    TUFS_STOP                   = 0x00,             // The focus status is stop 
    TUFS_FOCUSING               = 0x01,             // The focus status is focusing
    TUFS_COMPLETED              = 0x02,             // The focus status is completed
    TUFS_DEFOCUS                = 0x03,             // The focus status is defocus
}TUFOCUS_STATUS;

// typedef enum the multi roi status
typedef enum
{
	TUMR_DISABLE                = 0x00,             // The multi roi status is disable
	TUMR_SETPOS                 = 0x01,             // The multi roi status is set
	TUMR_ENABLE                 = 0x02,             // The multi roi status is enable
}TUMULTIROI_STATUS;

// typedef enum the math mode
typedef enum
{
	TUMATH_MODE_ADD             = 0x00,             // The math mode is add
	TUMATH_MODE_SUBTRACT        = 0x01,             // The math mode is subtract
	TUMATH_MODE_MULTIPLY        = 0x02,             // The math mode is multiply
	TUMATH_MODE_DIVIDE          = 0x03,             // The math mode is divide
}TUMATH_MODE;

// GeniCam  features

// element type
typedef enum
{
	TU_ElemValue                = 0x00,             //!< IValue interface
	TU_ElemBase                 = 0x01,             //!< IBase interface
	TU_ElemInteger              = 0x02,             //!< IInteger interface
	TU_ElemBoolean              = 0x03,             //!< IBoolean interface
	TU_ElemCommand              = 0x04,             //!< ICommand interface
	TU_ElemFloat                = 0x05,             //!< IFloat interface
	TU_ElemString               = 0x06,             //!< IString interface
	TU_ElemRegister             = 0x07,             //!< IRegister interface
	TU_ElemCategory             = 0x08,             //!< ICategory interface
	TU_ElemEnumeration          = 0x09,             //!< IEnumeration interface
	TU_ElemEnumEntry            = 0x0A,             //!< IEnumEntry interface
	TU_ElemPort                 = 0x0B,             //!< IPort interface
} TUELEM_TYPE;

//! access mode of a node
typedef enum
{
	TU_AM_NI                    = 0x00,              //!< Not implemented
	TU_AM_NA                    = 0x01,              //!< Not available
	TU_AM_WO                    = 0x02,              //!< Write Only
	TU_AM_RO                    = 0x03,              //!< Read Only
	TU_AM_RW                    = 0x04,              //!< Read and Write
}TUACCESS_MODE;

typedef enum
{
	TU_VS_Beginner              = 0x00,              //!< Always visible
	TU_VS_Expert                = 0x01,              //!< Visible for experts or Gurus
	TU_VS_Guru                  = 0x02,              //!< Visible for Gurus
	TU_VS_Invisible             = 0x03,              //!< Not Visible
	TU_VS_UndefinedVisibility   = 0x10,              //!< Object is not yet initialized
} TU_VISIBILITY;

typedef enum
{
	TU_REPRESENTATION_LINEAR      = 0x00,            // Slider with linear behaviour
	TU_REPRESENTATION_LOGARITHMIC = 0x01,            // Slider with logarithmic behaviour
	TU_REPRESENTATION_BOOLEAN     = 0x02,            // Checkbox
	TU_REPRESENTATION_PURE_NUMBER = 0x03,            // Decimal number in an edit control
	TU_REPRESENTATION_HEX_NUMBER  = 0x04,            // Hex number in an edit control
	TU_REPRESENTATION_IPV4ADDRESS = 0x05,            // IP address(IP version 4)
	TU_REPRESENTATION_MACADDRESS  = 0x06,            // MAC address
	TU_REPRESENTATION_UNDEFINDED  = 0x07,            // Undefinded Representation
} TU_REPRESENTATION;

typedef enum
{
	TU_CAMERA_XML                 = 0x00,            // [in] the device of the camera xml 
	TU_CAMERALINK_XML             = 0x01,            // [in] the device of the camera link xml 
} TUXML_DEVICE;

/* **************************************************************** *

    struct defines

* **************************************************************** */

class NVILen;

// the camera initialize struct
typedef struct _tagTUCAM_INIT
{
    UINT32  uiCamCount;                         // [out]
    PCHAR   pstrConfigPath;                     // [in] save the path of the camera parameters 
//  const TUCAM_GUID*	guid;					// [in ptr]
}TUCAM_INIT, *PTUCAM_INIT;

//  the camera open struct
typedef struct _tagTUCAM_OPEN
{
    UINT32  uiIdxOpen;                          // [in]
    HDTUCAM hIdxTUCam;                          // [out] the handle of the opened camera device   
}TUCAM_OPEN, *PTUCAM_OPEN;

//  the image open struct
typedef struct _tagTUIMG_OPEN
{
    PCHAR   pszfileName;                        // [in]  the full path of the image file
    HDTUIMG hIdxTUImg;                          // [out] the handle of the opened image file 
}TUIMG_OPEN, *PTUIMG_OPEN;

// the camera value text struct
typedef struct _tagTUCAM_VALUE_INFO
{   
    INT32   nID;                                // [in] TUCAM_IDINFO
    INT32   nValue;                             // [in] value of information
    PCHAR	pText;					            // [in/out] text of the value
    INT32	nTextSize;          				// [in] text buf size
}TUCAM_VALUE_INFO, *PTUCAM_VALUE_INFO;

// the camera value text struct
typedef struct _tagTUCAM_VALUE_TEXT
{   
    INT32   nID;                                // [in] TUCAM_IDPROP / TUCAM_IDCAPA
    DOUBLE  dbValue;                            // [in] value of property
    PCHAR	pText;					            // [in/out] text of the value
    INT32	nTextSize;          				// [in] text buf size
}TUCAM_VALUE_TEXT, *PTUCAM_VALUE_TEXT;

// the camera capability attribute
typedef struct _tagTUCAM_CAPA_ATTR
{
    INT32   idCapa;                             // [in] TUCAM_IDCAPA

    INT32   nValMin;                            // [out] minimum value
    INT32   nValMax;                            // [out] maximum value
    INT32   nValDft;                            // [out] default value
    INT32   nValStep;                           // [out] minimum stepping between a value and the next

}TUCAM_CAPA_ATTR, *PTUCAM_CAPA_ATTR;

// the camera property attribute
typedef struct _tagTUCAM_PROP_ATTR
{
    INT32   idProp;                             // [in] TUCAM_IDPROP
    INT32   nIdxChn;                            // [in/out] the index of channel

    DOUBLE  dbValMin;                           // [out] minimum value
    DOUBLE  dbValMax;                           // [out] maximum value
    DOUBLE  dbValDft;                           // [out] default value
    DOUBLE	dbValStep;                          // [out] minimum stepping between a value and the next

}TUCAM_PROP_ATTR, *PTUCAM_PROP_ATTR;

// the camera vendor property attribute
typedef struct _tagTUCAM_VPROP_ATTR
{
    INT32   idVProp;                            // [in] TUCAM_IDVPROP
    INT32   nIdxChn;                            // [in/out] the index of channel

    DOUBLE  dbValMin;                           // [out] minimum value
    DOUBLE  dbValMax;                           // [out] maximum value
    DOUBLE  dbValDft;                           // [out] default value
    DOUBLE	dbValStep;                          // [out] minimum stepping between a value and the next

}TUCAM_VPROP_ATTR, *PTUCAM_VPROP_ATTR;

// the camera process property attribute
typedef struct _tagTUCAM_PPROP_ATTR
{
    INT32   idPProp;                            // [in] TUCAM_IDVPROP
    INT32   procType;                           // [in] TUPROC_TYPE
 
    DOUBLE  dbValMin;                           // [out] minimum value
    DOUBLE  dbValMax;                           // [out] maximum value
    DOUBLE  dbValDft;                           // [out] default value
    DOUBLE	dbValStep;                          // [out] minimum stepping between a value and the next

}TUCAM_PPROP_ATTR, *PTUCAM_PPROP_ATTR;

// the camera roi attribute
typedef struct _tagTUCAM_ROI_ATTR
{
    BOOL    bEnable;                            // [in/out] The ROI enable

    INT32   nHOffset;                           // [in/out] The horizontal offset
    INT32   nVOffset;                           // [in/out] The vertical offset
    INT32   nWidth;                             // [in/out] The ROI width
    INT32   nHeight;                            // [in/out] The ROI height

}TUCAM_ROI_ATTR, *PTUCAM_ROI_ATTR;

// the camera multi roi attribute

// the camera size attribute
typedef struct _tagTUCAM_SIZE_ATTR
{
	INT32   nHOffset;                          // [in/out] The horizontal offset
	INT32   nVOffset;                          // [in/out] The vertical offset
	INT32   nWidth;                            // [in/out] The width
	INT32   nHeight;                           // [in/out] The height

}TUCAM_SIZE_ATTR, *PTUCAM_SIZE_ATTR;

typedef struct _tagTUCAM_MULTIROI_ATTR
{
	BOOL    bLimit;                             // [in/out] The ROI limit 256
	INT32   nROIStatus;                         // [in/out] The multi roi status
	TUCAM_SIZE_ATTR sizeAttr;                   // [in/out] The ROI size

}TUCAM_MULTIROI_ATTR, *PTUCAM_MULTIROI_ATTR;

// the camera roi calculate attribute
typedef struct _tagTUCAM_CALC_ROI_ATTR
{
    BOOL    bEnable;                            // [in/out] The ROI enable

    INT32   idCalc;                             // [in] TUCAM_IDCROI
    
    INT32   nHOffset;                           // [in/out] The horizontal offset
    INT32   nVOffset;                           // [in/out] The vertical offset
    INT32   nWidth;                             // [in/out] The ROI width
    INT32   nHeight;                            // [in/out] The ROI height

}TUCAM_CALC_ROI_ATTR, *PTUCAM_CALC_ROI_ATTR;

// the camera trigger attribute
typedef struct _tagTUCAM_TRIGGER_ATTR
{
	INT32   nTgrMode;                           // [in/out] The mode of trigger 
	INT32   nExpMode;                           // [in/out] The mode of exposure [0, 1] 0:Exposure time   1:Width level 
	INT32   nEdgeMode;                          // [in/out] The mode of edge     [0, 1] 0:Falling edge    1:Rising edge
	INT32   nDelayTm;                           // [in/out] The time delay
	INT32   nFrames;                            // [in/out] How many frames per trigger
	INT32   nBufFrames;                         // [in/out] How many frames in buffer
}TUCAM_TRIGGER_ATTR, *PTUCAM_TRIGGER_ATTR;

// the camera trigger out attribute
typedef struct _tagTUCAM_TRGOUT_ATTR
{
    INT32   nTgrOutPort;                        // [in/out] The port of trigger out 
    INT32   nTgrOutMode;                        // [in/out] The mode of trigger out
    INT32   nEdgeMode;                          // [in/out] The mode of edge     [0, 1] 1:Falling edge    0:Rising edge
    INT32   nDelayTm;                           // [in/out] The time delay
    INT32   nWidth;                             // [in/out] The width of pulse
}TUCAM_TRGOUT_ATTR, *PTUCAM_TRGOUT_ATTR;

// the camera any bin attribute
typedef struct _tagTUCAM_BIN_ATTR
{
	BOOL    bEnable;                            // [in/out] The any bin enable
	INT32   nMode;                              // [in/out] The any bin mode
	INT32   nWidth;                             // [in/out] The any bin width
	INT32   nHeight;                            // [in/out] The any bin height

}TUCAM_BIN_ATTR, *PTUCAM_BIN_ATTR;

// the camera frame struct
typedef struct _tagTUCAM_FRAME
{
    // TUCAM_Buf_WaitForFrame() use this structure. Some members have different direction.
    // [i:o] means, the member is input at TUCAM_Buf_WaitForFrame()
    // [i:i] and [o:o] means always input and output at both function.
    // "input" means application has to set the value before calling.
    // "output" means function fills a value at returning.

    CHAR szSignature[8];    // [out]Copyright+Version: TU+1.0 ['T', 'U', '1', '\0']

    //  The based information
    USHORT usHeader;        // [out] The frame header size
    USHORT usOffset;        // [out] The frame data offset
    USHORT usWidth;         // [out] The frame width
    USHORT usHeight;        // [out] The frame height
    UINT32 uiWidthStep;     // [out] The frame width step

    UCHAR  ucDepth;         // [out] The frame data depth 
    UCHAR  ucFormat;        // [out] The frame data format                  
    UCHAR  ucChannels;      // [out] The frame data channels
    UCHAR  ucElemBytes;     // [out] The frame data bytes per element
    UCHAR  ucFormatGet;     // [in]  Which frame data format do you want    see TUFRM_FORMATS

    UINT32 uiIndex;         // [in/out] The frame index number
    UINT32 uiImgSize;       // [out] The frame size
    UINT32 uiRsdSize;       // [in]  The frame reserved size    (how many frames do you want)
    UINT32 uiHstSize;       // [out] The frame histogram size

    PUCHAR pBuffer;         // [in/out] The frame buffer

} TUCAM_FRAME, *PTUCAM_FRAME;

// the file save struct
typedef struct _tagTUCAM_FILE_SAVE
{
    INT32   nSaveFmt;               // [in] the format of save file     see TUIMG_FORMATS
    PCHAR   pstrSavePath;           // [in] the path of save file 

    PTUCAM_FRAME pFrame;            // [in] the struct of camera frame
} TUCAM_FILE_SAVE, *PTUCAM_FILE_SAVE;

// the record save struct
typedef struct _tagTUCAM_REC_SAVE
{
    INT32   nCodec;                 // [in] the coder-decoder type
    PCHAR   pstrSavePath;           // [in] the path of save record file
    float   fFps;                   // [in] the current FPS
} TUCAM_REC_SAVE, *PTUCAM_REC_SAVE;

// the register read/write struct
typedef struct _tagTUCAM_REG_RW
{
    INT32   nRegType;               // [in] the format of register     see TUREG_TYPE

    PCHAR	pBuf;					// [in/out] pointer to the buffer value
    INT32	nBufSize;          	    // [in] the buffer size
} TUCAM_REG_RW, *PTUCAM_REG_RW;

//  typedef struct draw initialize
typedef struct _tagTUCAM_DRAW_INIT
{
#ifdef TUCAM_TARGETOS_IS_WIN32
    HWND    hWnd;                   // [in] Handle the draw window
#endif

    INT32   nMode;                  // [in] (The reserved)Whether use hardware acceleration (If the GPU support) default:TUDRAW_DFT
    UCHAR   ucChannels;             // [in] The data channels
    INT32   nWidth;                 // [in] The drawing data width
    INT32   nHeight;                // [in] The drawing data height

} TUCAM_DRAW_INIT, *PTUCAM_DRAW_INIT; 

//  typedef struct drawing
typedef struct _tagTUCAM_DRAW
{
    INT32   nSrcX;                  // [in/out] The x-coordinate, in pixels, of the upper left corner of the source rectangle.
    INT32   nSrcY;                  // [in/out] The y-coordinate, in pixels, of the upper left corner of the source rectangle.
    INT32   nSrcWidth;              // [in/out] Width,  in pixels, of the source rectangle.
    INT32   nSrcHeight;             // [in/out] Height, in pixels, of the source rectangle.

    INT32   nDstX;                  // [in/out] The x-coordinate, in MM_TEXT client coordinates, of the upper left corner of the destination rectangle.
    INT32   nDstY;                  // [in/out] The y-coordinate, in MM_TEXT client coordinates, of the upper left corner of the destination rectangle.
    INT32   nDstWidth;              // [in/out] Width,  in MM_TEXT client coordinates, of the destination rectangle.
    INT32   nDstHeight;             // [in/out] Height, in MM_TEXT client coordinates, of the destination rectangle.

    PTUCAM_FRAME pFrame;            // [in] the struct of camera frame

} TUCAM_DRAW, *PTUCAM_DRAW;

// the firmware update
typedef struct _tagTUCAM_FW_UPDATE
{
    INT32   nFwType;                // [in] the format of firmware     see TUFW_TYPE
    PCHAR   pstrFwFile;             // [in] the path of firmware file
} TUCAM_FW_UPDATE, *PTUCAM_FW_UPDATE;

// typedef struct point
typedef struct _tagTUCAM_POINT
{
    INT32 nPtX;
    INT32 nPtY;
}TUCAM_POINT, *PTUCAM_POINT;

// Define the struct of image header
typedef struct _tagTUCAM_IMG_HEADER
{
    CHAR szSignature[8];    // [out]Copyright+Version: TU+1.0 ['T', 'U', '1', '\0']

    //  The based information
    USHORT usHeader;        // [in/out] The image header size
    USHORT usOffset;        // [in/out] The image data offset
    USHORT usWidth;         // [in/out] The image width
    USHORT usHeight;        // [in/out] The image height
    UINT32 uiWidthStep;     // [in/out] The image width step

    UCHAR  ucDepth;         // [in/out] The image data depth (see from CV)
    UCHAR  ucFormat;        // [in/out] The image data format TUIMG_FORMAT
    UCHAR  ucChannels;      // [in/out] The image data channels
    UCHAR  ucElemBytes;     // [in/out] The image data bytes per element
    UCHAR  ucFormatGet;     // [in]     Which frame data format do you want

    UINT32 uiIndex;         // [out] The image index number
    UINT32 uiImgSize;       // [in/out] The image size
    UINT32 uiRsdSize;       // [in/out] The image reserved size
    UINT32 uiHstSize;       // [in/out] The image histogram size

    //  The data
    PUCHAR pImgData;        // [in/out] Pointer to the image data
    UINT32 *pImgHist;       // [in/out] Pointer to the image histogram data
    
    USHORT usLLevels;       // [out] The image left levels value
    USHORT usRLevels;       // [out] The image right levels value

    CHAR ucRsd1[64];        // The reserved

    DOUBLE dblExposure;     // [in/out] The exposure time

    CHAR   ucRsd2[170];     // The reserved

    DOUBLE dblTimeStamp;    // [in/out] The time stamp    
	DOUBLE dblTimeLast;     // [in/out] The time stamp last

	CHAR   ucRsd3[32];      // The reserved

	UCHAR  ucGPSTimeStampYear;  // [out] The GPS time stamp year
	UCHAR  ucGPSTimeStampMonth; // [out] The GPS time stamp month
	UCHAR  ucGPSTimeStampDay;   // [out] The GPS time stamp day
	UCHAR  ucGPSTimeStampHour;  // [out] The GPS time stamp hour
	UCHAR  ucGPSTimeStampMin;   // [out] The GPS time stamp min
	UCHAR  ucGPSTimeStampSec;   // [out] The GPS time stamp sec
	INT32  nGPSTimeStampNs;     // [out] The GPS time stamp ns

	DOUBLE dblConversionFactor; // [in/out] The conversion factor (DN)/ e-

#ifdef TUCAM_TARGETOS_IS_WIN32

#ifndef _WIN64
    CHAR   ucRsd4[647];     // The reserved
#else
    CHAR   ucRsd4[631];     // The reserved
#endif

#else
    CHAR   ucRsd4[631];     // The reserved
#endif

}TUCAM_IMG_HEADER, *PTUCAM_IMG_HEADER;

// Define the struct of image header
typedef struct _tagTUCAM_RAWIMG_HEADER
{
	USHORT usWidth;         // [in/out] The image width
	USHORT usHeight;        // [in/out] The image height

	USHORT usXOffset;       // [out] The buf x offset
	USHORT usYOffset;       // [out] The buf y offset
	USHORT usXPadding;      // [out] The buf x padding
	USHORT usYPadding;      // [out] The buf y padding
	USHORT usOffset;        // [out] The buf offset

	UCHAR  ucDepth;         // [in/out] The image data depth (see from CV)
	UCHAR  ucChannels;      // [in/out] The image data channels
	UCHAR  ucElemBytes;     // [in/out] The image data bytes per element

	UINT32 uiIndex;         // [out] The image index number
	UINT32 uiImgSize;       // [in/out] The image size
	UINT32 uiPixelFormat;   // [out] The buf pixel format

	DOUBLE dblExposure;     // [in/out] The exposure time

	//  The data
	PUCHAR pImgData;        // [in/out] Pointer to the image data

	DOUBLE dblTimeStamp;    // [in/out] The time stamp    
	DOUBLE dblTimeLast;     // [in/out] The time stamp last

}TUCAM_RAWIMG_HEADER, *PTUCAM_RAWIMG_HEADER;

// the subtract background struct
typedef struct _tagTUCAM_IMG_BACKGROUND
{
	BOOL                 bEnable;   // [in/out] The subtract background enable

	TUCAM_RAWIMG_HEADER  ImgHeader; // [in] the struct of camera frame
} TUCAM_IMG_BACKGROUND, *PTUCAM_IMG_BACKGROUND;

// the math struct
typedef struct _tagTUCAM_IMG_MATH
{
	BOOL                 bEnable;   // [in/out] The math enable
	
	INT                  nMode;     // [in/out] The math mode
	 
	USHORT               usGray;    // [in/out] The math gray
} TUCAM_IMG_MATH, *PTUCAM_IMG_MATH;

typedef struct _tagTUCAM_ELEMENT
{
	BYTE IsLocked;                 //[out] whether is locked
	BYTE Level;                    //[out] level
	DWORD Representation;           //[out] representation
	TUELEM_TYPE   Type;	           //[out] element type	[RO]
	TUACCESS_MODE Access;          //[out] access mode .write/read
	TU_VISIBILITY Visibility;      //[out] visibility
	INT32 nReserve;                //[out] reserve
	union {
		struct {
			INT64 nVal;	          //[in/out] current_value attribute or the length of string	[RO/WO/RW]
			INT64 nMin;	          //[out] minimum_value attribute or the length of string	[RO]
			INT64 nMax;	          //[out] maximum_value attribute or the length of string	[RO]
			INT64 nStep;	      //[out] increment attribute or the length of string		[RO]
			INT64 nDefault;	      //[out] old_value attribute or the length of string		[RO]
		};
		struct {
			DOUBLE dbVal;        //[in/out] current_value attribute [RO/WO/RW]
			DOUBLE dbMin;        //[out] minimum_value attribute [RO]
			DOUBLE dbMax;        //[out] maximum_value attribute [RO]
			DOUBLE dbStep;       //[out] increment attribute [RO]
			DOUBLE dbDefault;    //[out] old_value attribute [RO]
		};
	};
	PCHAR pName;                 //[out] id name	[RO]
	PCHAR pDisplayName;          //[out] display name	[RO]
	PCHAR pTransfer;		     //[out] string/Register address
	PCHAR pDesc;	             //[out] description
	PCHAR pUnit;	             //[out] unit
	PCHAR *pEntries;	         //[out] enumeration entry list
	INT64 PollingTime;           //[out] pollingTime
	INT64 DisplayPrecision;      //[out] displayPrecision 
}TUCAM_ELEMENT, *PTUCAM_ELEMENT;

typedef void(*BUFFER_CALLBACK)(void *pUserContex);

#endif  // _TUDEFINE_H_
