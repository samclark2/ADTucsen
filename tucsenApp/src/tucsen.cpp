/* This is a driver for the TUcsen Dhyana 900D camera. It should work for all
 * Tucsen USB3 cameras that use the TUCAM api.
 *
 * Author: David Vine
 * Date  : 28 October 2017
 *
 * Based on Mark Rivers PointGrey driver.
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <epicsExport.h>

#include <TUCamApi.h>

#include <ADDriver.h>
#include <iostream>

#define DRIVER_VERSION      0
#define DRIVER_REVISION     2
#define DRIVER_MODIFICATION 0

#define TucsenBusString           "T_BUS"     
#define TucsenProductIDString     "T_PRODUCT_ID"
#define TucsenDriverVersionString "T_DRIVER_VERSION"
#define TucsenTransferRateString  "T_TRANSER_RATE"
#define TucsenFrameFormatString   "T_FRAME_FORMAT"
#define TucsenBinningModeString   "T_BIN_MODE"
#define TucsenImageModeString     "T_IMG_MODE"
#define TucsenROIModeString       "T_ROI_MODE"
#define TucsenFanGearString       "T_FAN_GEAR"
#define TucsenTrigModeString      "T_TRIG_MODE"
#define TucsenOutTriggerPortString "T_OTRIG_PORT"
#define TucsenOutTriggerModeString "T_OTRIG_MODE"
#define TucsenOutTriggerEdgeString "T_OTRIG_EDG"
#define TucsenOutTriggerDelayString "T_OTRIG_DLY"
#define TucsenOutTriggerWidthString "T_OTRIG_WID"

static const int frameFormats[3] = {
    0x10,
    0x11,
    0x12
};

static const char* driverName = "tucsen";

static int TUCAMInitialized = 0;

/* Main driver class inherited from areaDetector ADDriver class */

class tucsen : public ADDriver
{
    public:
        tucsen( const char* portName, int cameraId, int traceMask, int maxBuffers,
                size_t maxMemory, int priority, int stackSize);
        // virtual ~tucsen();
        /* Virtual methods to override from ADDrive */
        virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);

        /* These should be private but must be called from C */
        void imageGrabTask();
        void shutdown();
        void tempTask();

    protected:
        int TucsenBus;
#define FIRST_TUCSEN_PARAM TucsenBus
        int TucsenProductID;
        int TucsenDriverVersion;
        int TucsenTransferRate;
        int TucsenBinMode;
        int TucsenFanGear;
        int TucsenImageMode;
        int TucsenROIMode;
        int TucsenFrameFormat;
        int TucsenTrigMode;
        int TucsenOutTriggerPort;
        int TucsenOutTriggerMode;
        int TucsenOutTriggerEdge;
        int TucsenOutTriggerDelay;
        int TucsenOutTriggerWidth;
#define LAST_TUCSEN_PARAM TucsenOutTriggerWidth

    private:
        /* Local methods to this class */
        asynStatus grabImage();
        asynStatus startCapture();
        asynStatus stopCapture();
        asynStatus setTrigger();

        asynStatus connectCamera();
        asynStatus disconnectCamera();

        /* camera property control functions */
        asynStatus getCamInfo(int nID, char* sBuf, int &val);
        asynStatus setCamInfo(int param, int nID, int dtype);
        asynStatus setSerialNumber();
        asynStatus setProperty(int nID, double value);
        asynStatus setCapability(int property, int value);
        asynStatus getCapability(int property, int& value);
        unsigned int checkStatus(unsigned int returnStatus);
        unsigned int mAcquiringData;

        /* Data */
        int cameraId_;
        int exiting_;
        TUCAM_INIT apiHandle_;
        TUCAM_OPEN camHandle_;
        TUCAM_FRAME frameHandle_;
        TUCAM_TRIGGER_ATTR triggerHandle_;
        TUCAM_TRGOUT_ATTR outTriggerHandle1_;
        TUCAM_TRGOUT_ATTR outTriggerHandle2_;
        TUCAM_TRGOUT_ATTR outTriggerHandle3_;
        epicsEventId startEventId_;
        NDArray *pRaw_;
};

#define NUM_TUCSEN_PARAMS ((int)(&LAST_TUCSEN_PARAM-&FIRST_TUCSEN_PARAM+1))


/* Configuration function to configure one camera
 *
 * This function needs to be called once for each camera used by the IOC. A
 * call to this function instantiates one object of the Tucsen class.
 * \param[in] portName asyn port to assign to the camera
 * \param[in] cameraId The camera index or serial number
 * \param[in] traceMask the initial value of asynTraceMask
 *            if set to 0 or 1 then asynTraceMask will be set to
 *            ASYN_TRACE_ERROR.
 *            if set to 0x21 ( ASYN_TRACE_WARNING | ASYN_TRACE_ERROR) then each
 *            call will be traced during initialization
 * \param[in] maxBuffers Maximum number of NDArray objects (image buffers) this
 *            driver is allowed to allocate.
 *            0 = unlimited
 * \param[in] maxMemory Maximum memort (in bytes) that this driver is allowed
 *            to allocate.
 *            0=unlimited
 * \param[in] priority The epics thread priority for this driver. 0= asyn
 *            default.
 * \param[in] stackSize The size of the stack of the EPICS port thread. 0=use
 *            asyn default.
 */
extern "C" int tucsenConfig(const char *portName, int cameraId, int traceMask,
        int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new tucsen( portName, cameraId, traceMask, maxBuffers, maxMemory, priority, stackSize);
    return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg tucsenDriverCreateArg0 = { "Port name", iocshArgString };
static const iocshArg tucsenDriverCreateArg1 = { "cameraId", iocshArgInt };
static const iocshArg tucsenDriverCreateArg2 = { "Trace Mask", iocshArgInt };
static const iocshArg tucsenDriverCreateArg3 = { "Max Buffers", iocshArgInt };
static const iocshArg tucsenDriverCreateArg4 = { "Max Memory", iocshArgInt };
static const iocshArg tucsenDriverCreateArg5 = { "priority", iocshArgInt };
static const iocshArg tucsenDriverCreateArg6 = { "Stack Size", iocshArgInt };
static const iocshArg* const tucsenDriverConfigArgs[] = { &tucsenDriverCreateArg0,
                                                          &tucsenDriverCreateArg1,
                                                          &tucsenDriverCreateArg2,
                                                          &tucsenDriverCreateArg3,
                                                          &tucsenDriverCreateArg4,
                                                          &tucsenDriverCreateArg5,
                                                          &tucsenDriverCreateArg6 };
static const iocshFuncDef configtucsenDriver = { "tucsenDriverCreate", 5, tucsenDriverConfigArgs };
static void configtucsenDriverCallFunc(const iocshArgBuf *args) {

    tucsenConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival,
                       args[5].ival, args[6].ival);

}

static void tucsenDriverRegister(void) {

    iocshRegister(&configtucsenDriver, configtucsenDriverCallFunc);

}

extern "C" {

    epicsExportRegistrar(tucsenDriverRegister);

}

static void c_shutdown(void *arg)
{
    tucsen *t = (tucsen *)arg;
    t->shutdown();
}

static void imageGrabTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->imageGrabTask();
}

static void tempReadTaskC(void *drvPvt)
{
    tucsen *t = (tucsen *)drvPvt;
    t->tempTask();
}

/* Constructor for the Tucsen class */

tucsen::tucsen(const char *portName, int cameraId, int traceMask, int maxBuffers,
        size_t maxMemory, int priority, int stackSize)
    : ADDriver( portName, 1, NUM_TUCSEN_PARAMS, maxBuffers, maxMemory, asynEnumMask,
            asynEnumMask, ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, priority, stackSize),
    cameraId_(cameraId), exiting_(0), pRaw_(NULL)
{
    static const char *functionName = "tucsen";

    asynStatus status;

    if(traceMask==0) traceMask = ASYN_TRACE_ERROR;
    pasynTrace->setTraceMask(pasynUserSelf, traceMask);

    createParam(TucsenBusString,           asynParamOctet,   &TucsenBus);
    createParam(TucsenProductIDString,     asynParamFloat64, &TucsenProductID);
    createParam(TucsenDriverVersionString, asynParamOctet,   &TucsenDriverVersion);
    createParam(TucsenTransferRateString,  asynParamFloat64, &TucsenTransferRate);
    createParam(TucsenBinningModeString,   asynParamInt32,   &TucsenBinMode);
    createParam(TucsenFanGearString,       asynParamInt32,   &TucsenFanGear);
    createParam(TucsenImageModeString,     asynParamInt32,   &TucsenImageMode);
    createParam(TucsenROIModeString,       asynParamInt32,   &TucsenROIMode);
    createParam(TucsenFrameFormatString,   asynParamInt32,   &TucsenFrameFormat);
    createParam(TucsenTrigModeString,       asynParamInt32,   &TucsenTrigMode);
    createParam(TucsenOutTriggerPortString, asynParamInt32, &TucsenOutTriggerPort);
    createParam(TucsenOutTriggerModeString, asynParamInt32, &TucsenOutTriggerMode);
    createParam(TucsenOutTriggerEdgeString, asynParamInt32, &TucsenOutTriggerEdge);
    createParam(TucsenOutTriggerDelayString, asynParamInt32, &TucsenOutTriggerDelay);
    createParam(TucsenOutTriggerWidthString, asynParamInt32, &TucsenOutTriggerWidth);

    /* Set initial values for some parameters */
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");
    setStringParam(ADManufacturer, "Tucsen");
    setIntegerParam(ADMaxSizeX, 2048);
    setIntegerParam(ADMaxSizeY, 2040);
    setIntegerParam(NDArraySizeX, 2048);
    setIntegerParam(NDArraySizeY, 2040);
    setIntegerParam(NDArraySize, 2*2048*2040);
    setStringParam(ADManufacturer, "Tucsen");

    status = connectCamera();
    setIntegerParam(TucsenOutTriggerPort, 0);
    setIntegerParam(TucsenOutTriggerMode, 0);
    setIntegerParam(TucsenOutTriggerEdge, 0);
    setIntegerParam(TucsenOutTriggerDelay, 1);
    setIntegerParam(TucsenOutTriggerWidth, 1);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: camera connection failed (%d)\n",
                driverName, functionName, status);
        report(stdout, 1);
        return;
    }

    startEventId_ = epicsEventCreate(epicsEventEmpty);

    /* Launch image read task */
    epicsThreadCreate("TucsenImageReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            imageGrabTaskC, this);

    /* Launch temp task */
    epicsThreadCreate("TucsenTempReadTask",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            tempReadTaskC, this);

    /* Launch shutdown task */
    epicsAtExit(c_shutdown, this);

    mAcquiringData = 0;

    return;
}

// tucsen::~tucsen(){
//     static const char *functionName = "~tucsen";
//     this->lock();
//     printf("%s::%s Shutdown and freeing up memory...\n", driverName, functionName);
//     shutdown();
//     this->unlock();
//     epicsThreadSleep(0.2);
// }

void tucsen::shutdown(void)
{
    std::cout<<"Shutting down driver"<<std::endl;
    exiting_=1;
    if (camHandle_.hIdxTUCam != NULL){
        disconnectCamera();
    }
    TUCAMInitialized--;
    if(TUCAMInitialized==0){
        TUCAM_Api_Uninit();
    }
}

asynStatus tucsen::connectCamera()
{
    static const char* functionName = "connectCamera";
    int tucStatus;
    asynStatus status;

    // Init API
    char szPath[1024] = {0};
    getcwd(szPath, 1024);
    apiHandle_.pstrConfigPath = szPath;
    apiHandle_.uiCamCount = 0;

    outTriggerHandle1_.nTgrOutPort=0;
    outTriggerHandle1_.nEdgeMode=0;
    outTriggerHandle1_.nDelayTm=1;
    outTriggerHandle1_.nWidth=1;
    outTriggerHandle1_.nTgrOutMode=0;
    outTriggerHandle2_.nTgrOutPort=1;
    outTriggerHandle2_.nEdgeMode=0;
    outTriggerHandle2_.nDelayTm=1;
    outTriggerHandle2_.nWidth=1;
    outTriggerHandle2_.nTgrOutMode=0;
    outTriggerHandle3_.nTgrOutPort=2;
    outTriggerHandle3_.nEdgeMode=0;
    outTriggerHandle3_.nDelayTm=1;
    outTriggerHandle3_.nWidth=1;
    outTriggerHandle3_.nTgrOutMode=0;

    tucStatus = TUCAM_Api_Init(&apiHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: TUCAM API init failed (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    if (apiHandle_.uiCamCount<1){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: no camera detected (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
    
    TUCAMInitialized++;
    
    // Init camera
    camHandle_.hIdxTUCam = NULL;
    camHandle_.uiIdxOpen = 0;

    tucStatus = TUCAM_Dev_Open(&camHandle_);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: open camera device failed (%d)\n",
                driverName, functionName, tucStatus);
        return asynError;
    }

    status = setCamInfo(TucsenBus, TUIDI_BUS, 0);
    status = setCamInfo(TucsenProductID, TUIDI_PRODUCT, 1);
    status = setCamInfo(ADSDKVersion, TUIDI_VERSION_API, 0);
    status = setCamInfo(ADFirmwareVersion, TUIDI_VERSION_FRMW, 0);
    status = setCamInfo(ADModel, TUIDI_CAMERA_MODEL, 0);
    status = setCamInfo(TucsenDriverVersion, TUIDI_VERSION_DRIVER, 0);
    status = setSerialNumber();

    return status;
}

asynStatus tucsen::disconnectCamera(void){
    static const char* functionName = "disconnectCamera";
    int tucStatus;
    int acquiring;
    asynStatus status;

    // check if acquiring
    status = getIntegerParam(ADAcquire, &acquiring);

    // if necessary stop acquiring
    if (status==asynSuccess && acquiring){
        tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: unable to stop acquisition (%d)\n",
                    driverName, functionName, tucStatus);
        }
    }

    // release buffer
    tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable to release camera buffer (%d)\n",
                driverName, functionName, tucStatus);
    }

    tucStatus = TUCAM_Dev_Close(camHandle_.hIdxTUCam);
    if (tucStatus!=TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unable close camera (%d)\n",
                driverName, functionName, tucStatus);
    }
    return asynSuccess;
}

void tucsen::imageGrabTask(void)
{
    static const char* functionName = "imageGrabTask";
    asynStatus status  = asynSuccess;
    int tucStatus;
    int imageCounter;
    int numImages, numImagesCounter;
    int imageMode;
    int arrayCallbacks;
    epicsTimeStamp startTime;
    int acquire;

    lock();

    while(1){
        /* Is acquisition active? */
        getIntegerParam(ADAcquire, &acquire);

        /* If we are not acquiring wait for a semaphore that is given when
         * acquisition is started */
        if(!mAcquiringData){
            setIntegerParam(ADStatus, ADStatusIdle);
            //callParamCallbacks();

            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: waiting for acquisition to start\n",
                    driverName, functionName);
            unlock();
			
            try{
                tucStatus = checkStatus(TUCAM_Cap_Stop(camHandle_.hIdxTUCam));
            } catch (const std::string &e) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n",
                    driverName, functionName, e.c_str());
                return;
            }
            epicsEventWait(startEventId_);
			printf("W lock\n");
            lock();
			printf("G lock\n");
			setIntegerParam(ADStatus, ADStatusWaiting);
			//callParamCallbacks();
			printf("Cap start\n");
            //tucStatus = TUCAM_Cap_Start(camHandle_.hIdxTUCam, TUCCM_SEQUENCE);
            status = setTrigger();
            try{
                tucStatus = checkStatus(TUCAM_Cap_Start(camHandle_.hIdxTUCam, triggerHandle_.nTgrMode));
            } catch (const std::string &e) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n",
                    driverName, functionName, e.c_str());
                return;
            }
			printf("Cap start done\n");
			setIntegerParam(ADStatus, ADStatusAcquire);
			printf("Call callback\n");
			//callParamCallbacks();
			printf("callback done\n");
            if (tucStatus!=TUCAMRET_SUCCESS){
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s:%s: Failed to start image capture (%d)\n",
                        driverName, functionName, tucStatus);
            }
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: acquisition started\n",
                    driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
        } else {
			setIntegerParam(ADStatus, ADStatusAcquire);
			callParamCallbacks();
		}


        /* Get the current time */
		printf("get time\n");
        epicsTimeGetCurrent(&startTime);

		printf("grab image\n");
        status = grabImage();
        if (status==asynError){
            // Release the allocated NDArray
            if (pRaw_) pRaw_->release();
            pRaw_ = NULL;
            continue;
        }

        getIntegerParam(NDArrayCounter, &imageCounter);
        getIntegerParam(ADNumImages, &numImages);
        getIntegerParam(ADNumImagesCounter, &numImagesCounter);
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
        imageCounter++;
        numImagesCounter++;
        setIntegerParam(NDArrayCounter, imageCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);

        if(arrayCallbacks){
            doCallbacksGenericPointer(pRaw_, NDArrayData, 0);
        }

        if (pRaw_) pRaw_->release();
        pRaw_ = NULL;

        if ((imageMode==ADImageSingle) || ((imageMode==ADImageMultiple) && (numImagesCounter>=numImages))){
            status = stopCapture();
        }
        callParamCallbacks();
    }
}

asynStatus tucsen::grabImage()
{
    static const char* functionName = "grabImage";
    asynStatus status = asynSuccess;
    int tucStatus;
    int header, offset;
    int nCols, nRows, stride;
    int bitDepth, pixelFormat, channels, pixelBytes;
    int index, dataSize;
    int tDataSize;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    int numColors;
    int pixelSize;
    size_t dims[3];
    int nDims;
    int count;

    try {
        printf("do unlock\n");
        unlock();
        printf("wait for buffer\n");
        int a;
        getIntegerParam(ADAcquire, &a);
        std::cout<<"ADAcqure is currently "<<a<<std::endl;
        std::cout<<"Setting ADAcquire to 1"<<std::endl;
        setIntegerParam(ADAcquire, 1);
        callParamCallbacks();

        tucStatus = checkStatus(TUCAM_Buf_WaitForFrame(camHandle_.hIdxTUCam, &frameHandle_));
    } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\n",
        driverName, functionName, e.c_str());
        status = asynError;
    }

    printf("Waiting for lock\n");
    lock();
    printf("Got lock\n");
    if (tucStatus!= TUCAMRET_SUCCESS){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Failed to wait for buffer (%d)\n",
                driverName, functionName, tucStatus);
    }

    setIntegerParam(ADStatus, ADStatusReadout);
    callParamCallbacks();
    header = frameHandle_.usHeader;
    offset = frameHandle_.usOffset;

    // Width & height are array dimensions
    nCols = frameHandle_.usWidth;
    nRows = frameHandle_.usHeight;

    // Not sure what this is "Frame image width step"?
    stride =frameHandle_.uiWidthStep;

    // Pixel bit depth
    bitDepth = frameHandle_.ucDepth;
    //std::cout<<"bitDepth: "<<bitDepth<<std::endl;
    // Image format
    pixelFormat = frameHandle_.ucFormat;
    // Number of channels
    channels = frameHandle_.ucChannels;
    //std::cout<<"Number of channels"<<channels<<std::endl;
    // Not sure what this is "Frame image data byte"?
    pixelBytes = frameHandle_.ucElemBytes;
    // Frame image serial number?
    index = frameHandle_.uiIndex;
    // Frame image data size
    tDataSize = frameHandle_.uiImgSize;

    /* There is zero documentation on what the formats mean
     * Most of the below is gleaned through trial and error */
    if (pixelFormat==16){
            // Raw data - no filtering applied
            dataType = NDUInt16;
            colorMode = NDColorModeMono;
            numColors = 1;
            pixelSize = 2;
    } else if (pixelFormat==17){
            if (bitDepth==1){
                dataType=NDUInt8;
                pixelSize = 1;
            } else if (bitDepth==2) {
                dataType=NDUInt16;
                pixelSize = 2;
            }
            
            if (channels==1){
                colorMode = NDColorModeMono;
                numColors = 1;
            } else if (channels==3){
                colorMode = NDColorModeRGB1;
                numColors = 3;
            }
    } else if (pixelFormat==18){
            dataType=NDUInt8;
            pixelSize = 1;
            colorMode = NDColorModeRGB1;
            numColors = 3;
    } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Unsupported pixel format %d\n",
                    driverName, functionName, pixelFormat);
            return asynError;
    }
    if (numColors==1){
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }

    dataSize = dims[0]*dims[1]*pixelSize;
    if (nDims==3) dataSize *= dims[2];

    if (dataSize != tDataSize){
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: data size mismatch: calculated=%d, reported=%d\n",
                driverName, functionName, dataSize, tDataSize);
        return asynError;
    }

    setIntegerParam(NDArraySizeX, nCols);
    setIntegerParam(NDArraySizeY, nRows);
    setIntegerParam(NDArraySize, (int)dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);

    pRaw_ = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if(!pRaw_){
        // No valid buffer so we need to abort
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s [%s] ERROR: Serious problem: not enough buffers left. Aborting acquisition!\n",
                driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
    }
    memcpy(pRaw_->pData, frameHandle_.pBuffer+offset, dataSize);
    getIntegerParam(NDArrayCounter, &count);
    pRaw_->uniqueId = count+1;
    // Changed the timestamping to match that of a DLS panda
    //updateTimeStamp(&pRaw_->epicsTS);
    //pRaw_->timeStamp = pRaw_->epicsTS.secPastEpoch+pRaw_->epicsTS.nsec/1e9;
    // Set the time stamp
    epicsTimeStamp arrayTime;
    epicsTimeGetCurrent(&arrayTime);
    pRaw_->timeStamp = arrayTime.secPastEpoch;
    pRaw_->timeStamp +=0.000000001*arrayTime.nsec;
    getAttributes(pRaw_->pAttributeList);

    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();
    pRaw_->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);

    return status;
}

/* Sets an int32 parameter */
asynStatus tucsen::writeInt32( asynUser *pasynUser, epicsInt32 value)
{
    static const char* functionName = "writeInt32";
    const char* paramName;
    asynStatus status = asynSuccess;
    int tucStatus;
    int function = pasynUser->reason;

    getParamName(function, &paramName);
    
    if (function==ADAcquire){
        if (value){
            mAcquiringData = 1;
            status = startCapture();
        } else {
            status = stopCapture();
        }
    } else{
        status = setIntegerParam(function, value);
        if (function==ADNumImages){
            //triggerHandle_.nFrames = value; // Trigger one frame(-1:to Ram)
        } else if (function==ADNumExposures){
            //status = setTrigger();
        } else if (function==ADTriggerMode){
            if (value==0){
                // internal
                triggerHandle_.nTgrMode = TUCCM_SEQUENCE; // Sequence mode
                std::cout<<"Setting internal Trigger (Sequence mode?)"<<std::endl;
            } else if (value==1){
                 //external
                triggerHandle_.nTgrMode = TUCCM_TRIGGER_STANDARD; // Exposure mode
                std::cout<<"Setting External trigger (Standard?)"<<std::endl;
            }
        } else if (function==TucsenFrameFormat){
            try{
                frameHandle_.ucFormatGet = frameFormats[value];
                frameHandle_.uiRsdSize = 1;
                frameHandle_.pBuffer = NULL;
                tucStatus = checkStatus(TUCAM_Buf_Release(camHandle_.hIdxTUCam));
                tucStatus = checkStatus(TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_));
            } catch (const std::string &e) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n",
                    driverName, functionName, e.c_str());
                return status;
            }
        } else if (function==TucsenBinMode){
            try {
                if ((value==0)||(value==1)){
                    status = setCapability(TUIDC_RESOLUTION, value);
                }
                frameHandle_.pBuffer = NULL;
                tucStatus = TUCAM_Buf_Release(camHandle_.hIdxTUCam);
                tucStatus = TUCAM_Buf_Alloc(camHandle_.hIdxTUCam, &frameHandle_);
            } catch (const std::string &e) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n",
                    driverName, functionName, e.c_str());
                return status;
            }
        } else if (function==TucsenFanGear){
            if ((value>=0)||(value<=6)){
                status = setCapability(TUIDC_FAN_GEAR, value);
            }
        } else if (function==TucsenImageMode){
            if (value < 3){
                status = setCapability(TUIDC_IMGMODESELECT, 0);
                std::cout<<"Setting IMGMode to 400BSIV1"<<std::endl;
            }
            else if (value == 3){
                status = setCapability(TUIDC_IMGMODESELECT, 1);
                std::cout<<"Setting IMGMode to V1 and V2)"<<std::endl;
            }
            else{
                status = setCapability(TUIDC_IMGMODESELECT, 2);
                std::cout<<"Setting IMGMode to 400BSIV2"<<std::endl;
            }
            if ((value == 0)||(value==3)||(value==4)){
                status = setCapability(TUIDP_GLOBALGAIN, 0);
                std::cout<<"Setting GainMode to CMS or HDR"<<std::endl;
            }
            else if ((value == 1)||(value==5)){
                status = setCapability(TUIDP_GLOBALGAIN, 1);
                std::cout<<"Setting IMGMode to HighGain"<<std::endl;
            }
            else{
                status = setCapability(TUIDP_GLOBALGAIN, 2);
                std::cout<<"Setting IMGMode to LowGain"<<std::endl;
            }


        } else if (function==TucsenROIMode){
            try{
                TUCAM_ROI_ATTR roiAttr;
                if (value==0){
                    std::cout<<"Setting Height to 2040"<<std::endl;
                    roiAttr.nHeight=2040;
                }
                else if (value==1){
                    std::cout<<"Setting Height to 1024"<<std::endl;
                    roiAttr.nHeight=1024;
                }
                else if (value==2){
                    std::cout<<"Setting Height to 512"<<std::endl;
                    roiAttr.nHeight=512;
                }
                else if (value==3){
                    std::cout<<"Setting Height to 256"<<std::endl;
                    roiAttr.nHeight=256;
                }
                else if (value==4){
                    std::cout<<"Setting Height to 128"<<std::endl;
                    roiAttr.nHeight=128;
                }
                else if (value==5){
                    std::cout<<"Setting Height to 64"<<std::endl;
                    roiAttr.nHeight=64;
                }
                roiAttr.nWidth=2048;
                roiAttr.bEnable = TRUE;
                checkStatus(TUCAM_Cap_SetROI(camHandle_.hIdxTUCam, roiAttr));
            }  catch (const std::string &e) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n",
                    driverName, functionName, e.c_str());
                return status;
            }

        } else if (function==TucsenOutTriggerPort){
            if (value==0){
                setIntegerParam(TucsenOutTriggerMode, outTriggerHandle1_.nTgrOutMode);
                setIntegerParam(TucsenOutTriggerEdge, outTriggerHandle1_.nEdgeMode);
                setIntegerParam(TucsenOutTriggerDelay, outTriggerHandle1_.nDelayTm);
                setIntegerParam(TucsenOutTriggerWidth, outTriggerHandle1_.nWidth);
            } else if (value==1){
                setIntegerParam(TucsenOutTriggerMode, outTriggerHandle2_.nTgrOutMode);
                setIntegerParam(TucsenOutTriggerEdge, outTriggerHandle2_.nEdgeMode);
                setIntegerParam(TucsenOutTriggerDelay, outTriggerHandle2_.nDelayTm);
                setIntegerParam(TucsenOutTriggerWidth, outTriggerHandle2_.nWidth);
            } else if (value==2){
                setIntegerParam(TucsenOutTriggerMode, outTriggerHandle3_.nTgrOutMode);
                setIntegerParam(TucsenOutTriggerEdge, outTriggerHandle3_.nEdgeMode);
                setIntegerParam(TucsenOutTriggerDelay, outTriggerHandle3_.nDelayTm);
                setIntegerParam(TucsenOutTriggerWidth, outTriggerHandle3_.nWidth);
            }
        } else if (function==TucsenOutTriggerMode){
            int port;
            getIntegerParam(TucsenOutTriggerPort, &port);
            if (port==0){
                outTriggerHandle1_.nTgrOutMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle1_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==1){
                outTriggerHandle2_.nTgrOutMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle2_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==2){
                outTriggerHandle3_.nTgrOutMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle3_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            }
        } else if (function==TucsenOutTriggerEdge){
            int port;
            getIntegerParam(TucsenOutTriggerPort, &port);
            if (port==0){
                outTriggerHandle1_.nEdgeMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle1_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==1){
                outTriggerHandle2_.nEdgeMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle2_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==2){
                outTriggerHandle3_.nEdgeMode=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle3_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            }
        } else if (function==TucsenOutTriggerDelay){
            int port;
            getIntegerParam(TucsenOutTriggerPort, &port);
            if (port==0){
                outTriggerHandle1_.nDelayTm=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle1_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==1){
                outTriggerHandle2_.nDelayTm=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle2_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==2){
                outTriggerHandle3_.nDelayTm=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle3_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            }
        } else if (function==TucsenOutTriggerWidth){
            int port;
            getIntegerParam(TucsenOutTriggerPort, &port);
            if (port==0){
                outTriggerHandle1_.nWidth=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle1_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==1){
                outTriggerHandle2_.nWidth=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle2_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            } else if (port==2){
                outTriggerHandle3_.nWidth=value;
                try {
                    tucStatus=checkStatus(TUCAM_Cap_SetTriggerOut(camHandle_.hIdxTUCam, outTriggerHandle3_));
                } catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s:%s: %s\n",
                              driverName, functionName, e.c_str());
                }
            }
        } else {
            if (function < FIRST_TUCSEN_PARAM){
                status = ADDriver::writeInt32(pasynUser, value);
            }
        }
    }
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
            "%s::%s function=%d, value=%d, status=%d\n",
            driverName, functionName, function, value, status);
    callParamCallbacks();
    return status;
}

asynStatus tucsen::setTrigger()
{
    static const char* functionName = "setTrigger";
    int extTrigger;
    int hwTriggerMode;
    try{
        triggerHandle_.nEdgeMode= TUCTD_RISING; // Stimulate rising edge
        triggerHandle_.nDelayTm = 0; // Delay 0 ms
        triggerHandle_.nFrames = 1;
        getCapability(ADTriggerMode,extTrigger);
        if (extTrigger==1){
            // Internal Trigger
            triggerHandle_.nExpMode = TUCTE_EXPTM;
        }
        else {
            getCapability(TucsenTrigMode, hwTriggerMode);
            if (hwTriggerMode == 0){
                // Gated
                triggerHandle_.nExpMode = TUCTE_EXPTM;
            } else if (hwTriggerMode == 1){
                // Timed
                triggerHandle_.nExpMode = TUCTE_WIDTH;
            }
        }
        TUCAM_Cap_SetTrigger(camHandle_.hIdxTUCam, triggerHandle_);
    } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\n",
            driverName, functionName, e.c_str());
    }
    //TUCAM_Cap_Start(m_opCam.hIdxCam, TUCCM_TRIGGER_STANDARD); // Standard trigger mode

 // Refer to memory management sample code for data obtaining

}

void tucsen::tempTask(void){
    static const char* functionName = "tempTask";
    TUCAM_VALUE_INFO valInfo;
    int tucStatus;
    double dbVal;

	lock();
    while (!exiting_){
        tucStatus = TUCAM_Prop_GetValue(camHandle_.hIdxTUCam, 
                TUIDP_TEMPERATURE, &dbVal);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to read temperature (%d)\n",
                    driverName, functionName, tucStatus);
        } else {
            setDoubleParam(ADTemperatureActual, dbVal);
        }

        valInfo.nID = TUIDI_TRANSFER_RATE;
        tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to read transfer rate(%d)\n",
                    driverName, functionName, tucStatus);
        } else {
            setDoubleParam(TucsenTransferRate, valInfo.nValue);
        }
        callParamCallbacks();
		unlock();
        epicsThreadSleep(0.5);
		lock();
    }
}

asynStatus tucsen::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char* functionName = "writeFloat64";
    const char* paramName;
	double valSec = 0.0;

    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
	int tucStatus =TUCAMRET_SUCCESS;

    status = setDoubleParam(function, value);

    if (function==ADAcquireTime){
		valSec = value*1000.0;
        tucStatus = setProperty(TUIDP_EXPOSURETM, valSec);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to set exposure time(%x)\n",
                    driverName, functionName, tucStatus);
		}
	} if (function==ADTemperature){
		/* Temperature is scaled -50C to 50C->0 to 100 */
		value = value+50.0;
        tucStatus = setProperty(TUIDP_TEMPERATURE, value);
        if (tucStatus!=TUCAMRET_SUCCESS){
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: failed to set temperature(%x)\n",
                    driverName, functionName, tucStatus);
		}
    } else {
        if (function < FIRST_TUCSEN_PARAM){
            status = ADDriver::writeFloat64(pasynUser, value);
        }
    }
    callParamCallbacks();
    return (asynStatus)status;
}

asynStatus tucsen::getCamInfo(int nID, char *sBuf, int &val)
{
    static const char* functionName = "getPropertyText";

    // Get camera information
    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    valInfo.pText = sBuf;
    valInfo.nTextSize = 1024;

    valInfo.nID = nID;
    tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
    if (tucStatus==TUCAMRET_SUCCESS){
        val = valInfo.nValue;
        return asynSuccess;
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: could not get %d\n",
                driverName, functionName, tucStatus);
        return asynError;
    }
}


unsigned int tucsen::checkStatus(unsigned int returnStatus)
{  

  char message[256];
  if (returnStatus == TUCAMRET_SUCCESS) {
    return returnStatus;
  } else if (returnStatus == TUCAMRET_FAILURE) {
    throw std::string("Generic error code, see API for more details. TUCAMRET_FAILURE");
  } else if (returnStatus == TUCAMRET_NO_MEMORY) {
    throw std::string("Not Enough Memory");
  } else if (returnStatus == TUCAMRET_NO_RESOURCE) {
    throw std::string("Not enough resources (Not including memory)");
  } else if (returnStatus == TUCAMRET_NO_MODULE) {
    throw std::string("No Sub module");

    //  initialization error
  } else if (returnStatus ==  TUCAMRET_NO_DRIVER) {
    throw std::string(" no driver");
  } else if (returnStatus ==  TUCAMRET_NO_CAMERA) {
    throw std::string(" no camera");
  } else if (returnStatus ==  TUCAMRET_NO_GRABBER         ) {
   throw std::string(" no grabber");  
  } else if (returnStatus ==  TUCAMRET_NO_PROPERTY        ) {
   throw std::string(" there is no alternative or influence id, or no more property id ");

  } else if (returnStatus ==  TUCAMRET_FAILOPEN_CAMERA    ) {
   throw std::string(" fail open the camera");
  } else if (returnStatus ==  TUCAMRET_FAILOPEN_BULKIN    ) {
   throw std::string(" fail open the bulk in endpoint");
  } else if (returnStatus ==  TUCAMRET_FAILOPEN_BULKOUT   ) {
   throw std::string(" fail open the bulk out endpoint");
  } else if (returnStatus ==  TUCAMRET_FAILOPEN_CONTROL   ) {
   throw std::string(" fail open the control endpoint");
  } else if (returnStatus ==  TUCAMRET_FAILCLOSE_CAMERA   ) {
   throw std::string(" fail close the camera");

  } else if (returnStatus ==  TUCAMRET_FAILOPEN_FILE      ) {
   throw std::string(" fail open the file");
  } else if (returnStatus ==  TUCAMRET_FAILOPEN_CODEC     ) {
   throw std::string(" fail open the video codec");
  } else if (returnStatus ==  TUCAMRET_FAILOPEN_CONTEXT   ) {
   throw std::string(" fail open the video context");

    //  status error
  } else if (returnStatus ==  TUCAMRET_INIT               ) {
   throw std::string(" API requires has not initialized state.");
  } else if (returnStatus ==  TUCAMRET_BUSY               ) {
   throw std::string(" API cannot process in busy state.");
  } else if (returnStatus ==  TUCAMRET_NOT_INIT           ) {
   throw std::string(" API requires has initialized state.");
  } else if (returnStatus ==  TUCAMRET_EXCLUDED           ) {
   throw std::string(" some resource is exclusive and already used.");
  } else if (returnStatus ==  TUCAMRET_NOT_BUSY           ) {
   throw std::string(" API requires busy state.");
  } else if (returnStatus ==  TUCAMRET_NOT_READY          ) {
   throw std::string(" API requires ready state.");
    
    //  wait error
  } else if (returnStatus ==  TUCAMRET_ABORT              ){     
    throw std::string(" abort process");
  } else if (returnStatus ==  TUCAMRET_TIMEOUT            ){     
    throw std::string(" timeout");
  } else if (returnStatus ==  TUCAMRET_LOSTFRAME          ){     
    throw std::string(" frame data is lost");
  } else if (returnStatus ==  TUCAMRET_MISSFRAME          ){     
    throw std::string(" frame is lost but reason is low lever driver's bug");
  } else if (returnStatus ==  TUCAMRET_USB_STATUS_ERROR   ){     
    throw std::string(" the USB status error");

    // calling error
  } else if (returnStatus ==  TUCAMRET_INVALID_CAMERA     ){     
    throw std::string(" invalid camera");
  } else if (returnStatus ==  TUCAMRET_INVALID_HANDLE     ){     
    throw std::string(" invalid camera handle");
  } else if (returnStatus ==  TUCAMRET_INVALID_OPTION     ){     
    throw std::string(" invalid the option value of structure");
  } else if (returnStatus ==  TUCAMRET_INVALID_IDPROP     ){     
    throw std::string(" invalid property id");
  } else if (returnStatus ==  TUCAMRET_INVALID_IDCAPA     ){     
    throw std::string(" invalid capability id");
  } else if (returnStatus ==  TUCAMRET_INVALID_IDPARAM    ){     
    throw std::string(" invalid parameter id");
  } else if (returnStatus ==  TUCAMRET_INVALID_PARAM      ){     
    throw std::string(" invalid parameter");
  } else if (returnStatus ==  TUCAMRET_INVALID_FRAMEIDX   ){     
    throw std::string(" invalid frame index");
  } else if (returnStatus ==  TUCAMRET_INVALID_VALUE      ){     
    throw std::string(" invalid property value");
  } else if (returnStatus ==  TUCAMRET_INVALID_EQUAL      ){     
    throw std::string(" invalid property value equal ");
  } else if (returnStatus ==  TUCAMRET_INVALID_CHANNEL    ){     
    throw std::string(" the property id specifies channel but channel is invalid");
  } else if (returnStatus ==  TUCAMRET_INVALID_SUBARRAY   ){     
    throw std::string(" the combination of subarray values are invalid. e.g. TUCAM_IDPROP_SUBARRAYHPOS + TUCAM_IDPROP_SUBARRAYHSIZE is greater than the number of horizontal pixel of sensor.");
  } else if (returnStatus ==  TUCAMRET_INVALID_VIEW       ){     
    throw std::string(" invalid view window handle");
  } else if (returnStatus ==  TUCAMRET_INVALID_PATH       ){     
    throw std::string(" invalid file path");
  } else if (returnStatus ==  TUCAMRET_INVALID_IDVPROP    ){     
    throw std::string(" invalid vendor property id");

  } else if (returnStatus ==  TUCAMRET_NO_VALUETEXT       ){     
    throw std::string(" the property does not have value text");
  } else if (returnStatus ==  TUCAMRET_OUT_OF_RANGE       ){     
    throw std::string(" value is out of range");

  } else if (returnStatus ==  TUCAMRET_NOT_SUPPORT        ){     
    throw std::string(" camera does not support the function or property with current settings");
  } else if (returnStatus ==  TUCAMRET_NOT_WRITABLE       ){     
    throw std::string(" the property is not writable ");
  } else if (returnStatus ==  TUCAMRET_NOT_READABLE       ){     
    throw std::string(" the property is not readable");

             
  } else if (returnStatus ==  TUCAMRET_WRONG_HANDSHAKE    ){     
    throw std::string(" this error happens TUCAM get error code from camera unexpectedly");
  } else if (returnStatus ==  TUCAMRET_NEWAPI_REQUIRED    ){     
    throw std::string(" old API does not support the value because only new API supports the value");
 
  } else if (returnStatus ==  TUCAMRET_ACCESSDENY         ){     
    throw std::string(" the property cannot access during this TUCAM status");

  } else if (returnStatus ==  TUCAMRET_NO_CORRECTIONDATA  ){     
    throw std::string(" not take the dark and shading correction data yet.");

  } else if (returnStatus ==  TUCAMRET_INVALID_PRFSETS    ){     
    throw std::string(" the profiles set name is invalid");
  } else if (returnStatus ==  TUCAMRET_INVALID_IDPPROP    ){     
    throw std::string(" invalid process property id");

  } else if (returnStatus ==  TUCAMRET_DECODE_FAILURE    ){
    throw std::string(" the image decoding raw data to rgb data failure ");
  } else if (returnStatus ==  TUCAMRET_COPYDATA_FAILURE  ){
    throw std::string(" the image data copying failure ");
  } else if (returnStatus ==  TUCAMRET_ENCODE_FAILURE    ){
    throw std::string(" the image encoding data  to video failure ");
  } else if (returnStatus ==  TUCAMRET_WRITE_FAILURE     ){
    throw std::string(" the write the video frame failure ");

    //  camera or bus trouble
  } else if (returnStatus ==  TUCAMRET_FAIL_READ_CAMERA  ){
    throw std::string(" fail read from camera   ");
  } else if (returnStatus ==  TUCAMRET_FAIL_WRITE_CAMERA ){
    throw std::string(" fail write to camera ");
  } else if (returnStatus ==  TUCAMRET_OPTICS_UNPLUGGED  ){
    throw std::string(" optics part is unplugged so please check it. ");

  } else if (returnStatus ==  TUCAMRET_RECEIVE_FINISH    ){
    throw std::string(" no error, vendor receive frame message   ");
  } else if (returnStatus ==  TUCAMRET_EXTERNAL_TRIGGER  ){
    throw std::string(" no error, receive the external trigger signal ");
  } else {
    sprintf(message, "ERROR: Unknown error code=%d returned from Tucsen SDK.", returnStatus);
    throw std::string(message);
  }

  return returnStatus;
}

asynStatus tucsen::setCamInfo(int param, int nID, int dtype)
{
    static const char* functionName = "setCamInfo";

    int tucStatus;
    TUCAM_VALUE_INFO valInfo;
    int sSize = 1024;
    char sInfo[1024] = {0};
    valInfo.pText = sInfo;
    valInfo.nTextSize = sSize;
    valInfo.nID = nID;

    tucStatus = TUCAM_Dev_GetInfo(camHandle_.hIdxTUCam, &valInfo);
    if (tucStatus==TUCAMRET_SUCCESS){
        if (param==TucsenBus){
            if (valInfo.nValue==768){
                setStringParam(TucsenBus, "USB3.0");
            } else{
                setStringParam(TucsenBus, "USB2.0");
            }
        } else if (dtype==0){
            setStringParam(param, valInfo.pText);
        } else if (dtype==1){
            setDoubleParam(param, valInfo.nValue);
        } else if (dtype==2){
            setIntegerParam(param, valInfo.nValue);
        }
        callParamCallbacks();
        return asynSuccess;
    } else {
        return asynError;
    }
}

asynStatus tucsen::setSerialNumber()
{
    static const char* functionName = "setSerialNumber";
    int tucStatus;
    char cSN[TUSN_SIZE] = {0};
    TUCAM_REG_RW regRW;

    regRW.nRegType = TUREG_SN;
    regRW.pBuf = &cSN[0];
    regRW.nBufSize = TUSN_SIZE;

    tucStatus = TUCAM_Reg_Read(camHandle_.hIdxTUCam, regRW);
    if (tucStatus==TUCAMRET_SUCCESS){
        setStringParam(ADSerialNumber, cSN);
        return asynSuccess;
    } else {
        return asynError;
    }
}

asynStatus tucsen::setProperty(int property, double value){

    static const char* functionName = "setProperty";
    TUCAM_PROP_ATTR attrProp;
    int tucStatus;

    attrProp.nIdxChn = 0;
    attrProp.idProp = property;
    try {
        tucStatus = checkStatus(TUCAM_Prop_GetAttr(camHandle_.hIdxTUCam, &attrProp));
        if (tucStatus==TUCAMRET_SUCCESS)
        {
    		printf("min: %f Max: %f\n", attrProp.dbValMin, attrProp.dbValMax);
            if(value<attrProp.dbValMin){
                value = attrProp.dbValMin;
    			printf("Clipping set min value: %d, %f\n", property, value);
            } else if (value>attrProp.dbValMax){
                value = attrProp.dbValMax;
    			printf("Clipping set max value: %d, %f\n", property, value);
            }
        }
    	printf("Value: %f\n", value);
        tucStatus = checkStatus(TUCAM_Prop_SetValue(camHandle_.hIdxTUCam, property, value));
    }  catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s unable to set property %d\n",
            driverName, functionName, property);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\n",
            driverName, functionName, e.c_str());
        return (asynStatus)tucStatus;
    }
    return (asynStatus)tucStatus;
}

asynStatus tucsen::setCapability(int property, int val)
{
    static const char* functionName = "setCapability";
    int tucStatus;
	TUCAM_CAPA_ATTR attrCapa;
	TUCAM_VALUE_TEXT valText;
	char szRes[64] = {0};
	valText.nTextSize = 64;
	valText.pText = &szRes[0];
	attrCapa.idCapa = property;

    try{
        tucStatus = checkStatus(TUCAM_Capa_GetAttr(camHandle_.hIdxTUCam, &attrCapa));
	    if (tucStatus==TUCAMRET_SUCCESS){
	   	    int nCnt = attrCapa.nValMax - attrCapa.nValMin;
	       	valText.nID = property;

	   	    for (int i=0; i<nCnt;i++){
	   		   valText.dbValue = i;
               try{
	   		      checkStatus(TUCAM_Capa_GetValueText(camHandle_.hIdxTUCam, &valText));
	   		   }catch (const std::string &e) {
                    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: %s\n TUCAM_CAPA_GetValueText\n",
                    driverName, functionName, e.c_str());
               }
               printf("%s\n", valText.pText);
	   	   }
        }
    } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\nTUCAM_Capa_GetAttr\n",
            driverName, functionName, e.c_str());
    }
    try{
        tucStatus = checkStatus(TUCAM_Capa_SetValue(camHandle_.hIdxTUCam, property, val));
        if (tucStatus!=TUCAMRET_SUCCESS)
        {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s unable to set capability %d=%d\n",
                    driverName, functionName, property, val);
            return asynError;
        }
    } catch (const std::string &e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: %s\nTUCAM_Capa_SetValue\n",
            driverName, functionName, e.c_str());
        return asynError;
    }

    return asynSuccess;
}

asynStatus tucsen::getCapability(int property, int& val)
{
    static const char* functionName = "getCapability";
    int tucStatus;

    tucStatus = TUCAM_Capa_GetValue(camHandle_.hIdxTUCam, property, &val);
    if (tucStatus!=TUCAMRET_SUCCESS)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s unable to get capability %d=%d\n",
                driverName, functionName, property, val);
        return asynError;
    }
    return asynSuccess;
}


asynStatus tucsen::startCapture()
{
    static const char* functionName = "startCapture";
    int a;
    getIntegerParam(ADAcquire, &a);
    std::cout<<"ADACQUIRE om startCapture"<<a<<std::endl;

    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    epicsEventSignal(startEventId_);
    return asynSuccess;
}

asynStatus tucsen::stopCapture()
{
    static const char* functionName = "stopCapture";
    int tucStatus;
    std::cout<<"Stopping Capture"<<std::endl;
    TUCAM_Buf_AbortWait(camHandle_.hIdxTUCam);
    setShutter(0);
    setIntegerParam(ADStatus, ADStatusWaiting);
    callParamCallbacks();
    //tucStatus = TUCAM_Cap_Stop(camHandle_.hIdxTUCam);
    setIntegerParam(ADAcquire, 0);
    setIntegerParam(ADStatus, ADStatusIdle);
    callParamCallbacks();
    mAcquiringData = 0;
    return asynSuccess;
}

static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"CameraId", iocshArgInt};
static const iocshArg configArg2 = {"traceMask", iocshArgInt};
static const iocshArg configArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg configArg4 = {"maxMemory", iocshArgInt};
static const iocshArg configArg5 = {"priority", iocshArgInt};
static const iocshArg configArg6 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs [] = {&configArg0,
                                               &configArg1,
                                               &configArg2,
                                               &configArg3,
                                               &configArg4,
                                               &configArg5,
                                               &configArg6};
static const iocshFuncDef configtucsen = {"tucsenConfig", 7, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    tucsenConfig(args[0].sval, args[1].ival, args[2].ival,
                 args[3].ival, args[4].ival, args[5].ival,
                 args[6].ival);
}

static void tucsenRegister(void)
{
    iocshRegister(&configtucsen, configCallFunc);
}

extern "C" {
    epicsExportRegistrar(tucsenRegister);
}

