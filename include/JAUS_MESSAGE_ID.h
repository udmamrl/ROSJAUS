////////////////////////////////////////////////////////////////////////////////////
/// JAUS_MESSAGES_ID.h
///
/// The meaasge ID in
///   AS5710 JAUS Core Service Set
///   AS6009 JAUS Mobility Service Set (Draft ver:11FEB09)
///
////////////////////////////////////////////////////////////////////////////////////

#ifndef __JAUS_MESSAGES_ID_INCLUDE__H
#define __JAUS_MESSAGES_ID_INCLUDE__H

// AS5710 JAUS Core Service Set
// CommandClass
const unsigned short JAUS_ID_SetAuthority              = 0x0001  ; //ID 0001h
const unsigned short JAUS_ID_Shutdown                  = 0x0002  ; //ID 0002h
const unsigned short JAUS_ID_Standby                   = 0x0003  ; //ID 0003h
const unsigned short JAUS_ID_Resume                    = 0x0004  ; //ID 0004h
const unsigned short JAUS_ID_Reset                     = 0x0005  ; //ID 0005h
const unsigned short JAUS_ID_SetEmergency              = 0x0006  ; //ID 0006h
const unsigned short JAUS_ID_ClearEmergency            = 0x0007  ; //ID 0007h
const unsigned short JAUS_ID_RequestControl            = 0x000D  ; //ID 000Dh
const unsigned short JAUS_ID_ReleaseControl            = 0x000E  ; //ID 000Eh
const unsigned short JAUS_ID_ConfirmControl            = 0x000F  ; //ID 000Fh
const unsigned short JAUS_ID_RejectControl             = 0x0010  ; //ID 0010h
const unsigned short JAUS_ID_SetTime                   = 0x0011  ; //ID 0011h
const unsigned short JAUS_ID_CreateEvent               = 0x01F0  ; //ID 01F0h
const unsigned short JAUS_ID_UpdateEvent               = 0x01F1  ; //ID 01F1h
const unsigned short JAUS_ID_CancelEvent               = 0x01F2  ; //ID 01F2h
const unsigned short JAUS_ID_ConfirmEventRequest       = 0x01F3  ; //ID 01F3h
const unsigned short JAUS_ID_RejectEventRequest        = 0x01F4  ; //ID 01F4h
const unsigned short JAUS_ID_RegisterServices          = 0x0B00  ; //ID 0B00h
// QueryClass
const unsigned short JAUS_ID_QueryAuthority            = 0x2001  ; //ID 2001h
const unsigned short JAUS_ID_QueryStatus               = 0x2002  ; //ID 2002h
const unsigned short JAUS_ID_QueryTimeout              = 0x2003  ; //ID 2003h
const unsigned short JAUS_ID_QueryTime                 = 0x2011  ; //ID 2011h
const unsigned short JAUS_ID_QueryControl              = 0x200D  ; //ID 200Dh
const unsigned short JAUS_ID_QueryEvents               = 0x21F0  ; //ID 21F0h
const unsigned short JAUS_ID_QueryHeartbeatPulse       = 0x2202  ; //ID 2202h
const unsigned short JAUS_ID_QueryIdentification       = 0x2B00  ; //ID 2B00h
const unsigned short JAUS_ID_QueryConfiguration        = 0x2B01  ; //ID 2B01h
const unsigned short JAUS_ID_QuerySubsystemList        = 0x2B02  ; //ID 2B02h
const unsigned short JAUS_ID_QueryServices             = 0x2B03  ; //ID 2B03h
// InformClass
const unsigned short JAUS_ID_ReportAuthority           = 0x4001  ; //ID 4001h
const unsigned short JAUS_ID_ReportStatus              = 0x4002  ; //ID 4002h
const unsigned short JAUS_ID_ReportTimeout             = 0x4003  ; //ID 4003h
const unsigned short JAUS_ID_ReportTime                = 0x4011  ; //ID 4011h
const unsigned short JAUS_ID_ReportControl             = 0x400D  ; //ID 400Dh
const unsigned short JAUS_ID_ReportEvents              = 0x41F0  ; //ID 41F0h
const unsigned short JAUS_ID_Event                     = 0x41F1  ; //ID 41F1h
const unsigned short JAUS_ID_ReportHeartbeatPulse      = 0x4202  ; //ID 4202h
const unsigned short JAUS_ID_ReportIdentification      = 0x4B00  ; //ID 4B00h
const unsigned short JAUS_ID_ReportConfiguration       = 0x4B01  ; //ID 4B01h
const unsigned short JAUS_ID_ReportSubsystemList       = 0x4B02  ; //ID 4B02h
const unsigned short JAUS_ID_ReportServices            = 0x4B03  ; //ID 4B03h

// AS6009 JAUS Mobility Service Set (Draft ver:11FEB09)
// CommandClass
const unsigned short JAUS_ID_SetGlobalPose             = 0x0402  ; //ID 0402h
const unsigned short JAUS_ID_SetLocalPose              = 0x0403  ; //ID 0403h
const unsigned short JAUS_ID_SetWrenchEffort           = 0x0405  ; //ID 0405h
const unsigned short JAUS_ID_SetGlobalVector           = 0x0407  ; //ID 0407h
const unsigned short JAUS_ID_SetLocalVector            = 0x0408  ; //ID 0408h
const unsigned short JAUS_ID_SetTravelSpeed            = 0x040A  ; //ID 040Ah
const unsigned short JAUS_ID_SetGlobalWaypoint         = 0x040C  ; //ID 040Ch
const unsigned short JAUS_ID_SetLocalWaypoint          = 0x040D  ; //ID 040Dh
const unsigned short JAUS_ID_SetGlobalPathSegment      = 0x040F  ; //ID 040Fh
const unsigned short JAUS_ID_SetLocalPathSegment       = 0x0410  ; //ID 0410h
const unsigned short JAUS_ID_SetGeomagneticProperty    = 0x0412  ; //ID 0412h
const unsigned short JAUS_ID_SetVelocityCommand        = 0x0415  ; //ID 0415h
const unsigned short JAUS_ID_SetAccelerationLimit      = 0x0416  ; //ID 0416h
const unsigned short JAUS_ID_SetElement                = 0x041A  ; //ID 041Ah
const unsigned short JAUS_ID_DeleteElement             = 0x041B  ; //ID 041Bh
const unsigned short JAUS_ID_ConfirmElementRequest     = 0x041C  ; //ID 041Ch
const unsigned short JAUS_ID_RejectElementRequest      = 0x041D  ; //ID 041Dh
const unsigned short JAUS_ID_ExecuteList               = 0x041E  ; //ID 041Eh
// QueryClass
const unsigned short JAUS_ID_QueryGlobalPose           = 0x2402  ; //ID 2402h
const unsigned short JAUS_ID_QueryLocalPose            = 0x2403  ; //ID 2403h
const unsigned short JAUS_ID_QueryVelocityState        = 0x2404  ; //ID 2404h
const unsigned short JAUS_ID_QueryWrenchEffort         = 0x2405  ; //ID 2405h
const unsigned short JAUS_ID_QueryGlobalVector         = 0x2407  ; //ID 2407h
const unsigned short JAUS_ID_QueryLocalVector          = 0x2408  ; //ID 2408h
const unsigned short JAUS_ID_QueryTravelSpeed          = 0x240A  ; //ID 240Ah
const unsigned short JAUS_ID_QueryGlobalWaypoint       = 0x240C  ; //ID 240Ch
const unsigned short JAUS_ID_QueryLocalWaypoint        = 0x240D  ; //ID 240Dh
const unsigned short JAUS_ID_QueryGlobalPathSegment    = 0x240F  ; //ID 240Fh
const unsigned short JAUS_ID_QueryLocalPathSegment     = 0x2410  ; //ID 2410h
const unsigned short JAUS_ID_QueryGeomagneticProperty  = 0x2412  ; //ID 2412h
const unsigned short JAUS_ID_QueryVelocityCommand      = 0x2415  ; //ID 2415h
const unsigned short JAUS_ID_QueryAccelerationLimit    = 0x2416  ; //ID 2416h
const unsigned short JAUS_ID_QueryAccelerationState    = 0x2417  ; //ID 2417h
const unsigned short JAUS_ID_QueryElement              = 0x241A  ; //ID 241Ah
const unsigned short JAUS_ID_QueryElementList          = 0x241B  ; //ID 241Bh
const unsigned short JAUS_ID_QueryElementCount         = 0x241C  ; //ID 241Ch
const unsigned short JAUS_ID_QueryActiveElement        = 0x241E  ; //ID 241Eh
// InformClass
const unsigned short JAUS_ID_ReportGlobalPose          = 0x4402  ; //ID 4402h
const unsigned short JAUS_ID_ReportLocalPose           = 0x4403  ; //ID 4403h
const unsigned short JAUS_ID_ReportVelocityState       = 0x4404  ; //ID 4404h
const unsigned short JAUS_ID_ReportWrenchEffort        = 0x4405  ; //ID 4405h
const unsigned short JAUS_ID_ReportGlobalVector        = 0x4407  ; //ID 4407h
const unsigned short JAUS_ID_ReportLocalVector         = 0x4408  ; //ID 4408h
const unsigned short JAUS_ID_ReportTravelSpeed         = 0x440A  ; //ID 440Ah
const unsigned short JAUS_ID_ReportGlobalWaypoint      = 0x440C  ; //ID 440Ch
const unsigned short JAUS_ID_ReportLocalWaypoint       = 0x440D  ; //ID 440Dh
const unsigned short JAUS_ID_ReportGlobalPathSegment   = 0x440F  ; //ID 440Fh
const unsigned short JAUS_ID_ReportLocalPathSegment    = 0x4410  ; //ID 4410h
const unsigned short JAUS_ID_ReportGeomagneticProperty = 0x4412  ; //ID 4412h
const unsigned short JAUS_ID_ReportVelocityCommand     = 0x4415  ; //ID 4415h
const unsigned short JAUS_ID_ReportAccelerationLimit   = 0x4416  ; //ID 4416h
const unsigned short JAUS_ID_ReportAccelerationState   = 0x4417  ; //ID 4417h
const unsigned short JAUS_ID_ReportElement             = 0x441A  ; //ID 441Ah
const unsigned short JAUS_ID_ReportElementList         = 0x441B  ; //ID 441Bh
const unsigned short JAUS_ID_ReportElementCount        = 0x441C  ; //ID 441Ch
const unsigned short JAUS_ID_ReportActiveElement       = 0x441E  ; //ID 441Eh
// AS5710 JAUS Core Service Set
// CommandClass
const unsigned short SetAuthority              = 0x0001  ; //ID 0001h
const unsigned short Shutdown                  = 0x0002  ; //ID 0002h
const unsigned short Standby                   = 0x0003  ; //ID 0003h
const unsigned short Resume                    = 0x0004  ; //ID 0004h
const unsigned short Reset                     = 0x0005  ; //ID 0005h
const unsigned short SetEmergency              = 0x0006  ; //ID 0006h
const unsigned short ClearEmergency            = 0x0007  ; //ID 0007h
const unsigned short RequestControl            = 0x000D  ; //ID 000Dh
const unsigned short ReleaseControl            = 0x000E  ; //ID 000Eh
const unsigned short ConfirmControl            = 0x000F  ; //ID 000Fh
const unsigned short RejectControl             = 0x0010  ; //ID 0010h
const unsigned short SetTime                   = 0x0011  ; //ID 0011h
const unsigned short CreateEvent               = 0x01F0  ; //ID 01F0h
const unsigned short UpdateEvent               = 0x01F1  ; //ID 01F1h
const unsigned short CancelEvent               = 0x01F2  ; //ID 01F2h
const unsigned short ConfirmEventRequest       = 0x01F3  ; //ID 01F3h
const unsigned short RejectEventRequest        = 0x01F4  ; //ID 01F4h
const unsigned short RegisterServices          = 0x0B00  ; //ID 0B00h
// QueryClass
const unsigned short QueryAuthority            = 0x2001  ; //ID 2001h
const unsigned short QueryStatus               = 0x2002  ; //ID 2002h
const unsigned short QueryTimeout              = 0x2003  ; //ID 2003h
const unsigned short QueryTime                 = 0x2011  ; //ID 2011h
const unsigned short QueryControl              = 0x200D  ; //ID 200Dh
const unsigned short QueryEvents               = 0x21F0  ; //ID 21F0h
const unsigned short QueryHeartbeatPulse       = 0x2202  ; //ID 2202h
const unsigned short QueryIdentification       = 0x2B00  ; //ID 2B00h
const unsigned short QueryConfiguration        = 0x2B01  ; //ID 2B01h
const unsigned short QuerySubsystemList        = 0x2B02  ; //ID 2B02h
const unsigned short QueryServices             = 0x2B03  ; //ID 2B03h
// InformClass
const unsigned short ReportAuthority           = 0x4001  ; //ID 4001h
const unsigned short ReportStatus              = 0x4002  ; //ID 4002h
const unsigned short ReportTimeout             = 0x4003  ; //ID 4003h
const unsigned short ReportTime                = 0x4011  ; //ID 4011h
const unsigned short ReportControl             = 0x400D  ; //ID 400Dh
const unsigned short ReportEvents              = 0x41F0  ; //ID 41F0h
const unsigned short Event                     = 0x41F1  ; //ID 41F1h
const unsigned short ReportHeartbeatPulse      = 0x4202  ; //ID 4202h
const unsigned short ReportIdentification      = 0x4B00  ; //ID 4B00h
const unsigned short ReportConfiguration       = 0x4B01  ; //ID 4B01h
const unsigned short ReportSubsystemList       = 0x4B02  ; //ID 4B02h
const unsigned short ReportServices            = 0x4B03  ; //ID 4B03h

// AS6009 JAUS Mobility Service Set (Draft ver:11FEB09)
// CommandClass
const unsigned short SetGlobalPose             = 0x0402  ; //ID 0402h
const unsigned short SetLocalPose              = 0x0403  ; //ID 0403h
const unsigned short SetWrenchEffort           = 0x0405  ; //ID 0405h
const unsigned short SetGlobalVector           = 0x0407  ; //ID 0407h
const unsigned short SetLocalVector            = 0x0408  ; //ID 0408h
const unsigned short SetTravelSpeed            = 0x040A  ; //ID 040Ah
const unsigned short SetGlobalWaypoint         = 0x040C  ; //ID 040Ch
const unsigned short SetLocalWaypoint          = 0x040D  ; //ID 040Dh
const unsigned short SetGlobalPathSegment      = 0x040F  ; //ID 040Fh
const unsigned short SetLocalPathSegment       = 0x0410  ; //ID 0410h
const unsigned short SetGeomagneticProperty    = 0x0412  ; //ID 0412h
const unsigned short SetVelocityCommand        = 0x0415  ; //ID 0415h
const unsigned short SetAccelerationLimit      = 0x0416  ; //ID 0416h
const unsigned short SetElement                = 0x041A  ; //ID 041Ah
const unsigned short DeleteElement             = 0x041B  ; //ID 041Bh
const unsigned short ConfirmElementRequest     = 0x041C  ; //ID 041Ch
const unsigned short RejectElementRequest      = 0x041D  ; //ID 041Dh
const unsigned short ExecuteList               = 0x041E  ; //ID 041Eh
// QueryClass
const unsigned short QueryGlobalPose           = 0x2402  ; //ID 2402h
const unsigned short QueryLocalPose            = 0x2403  ; //ID 2403h
const unsigned short QueryVelocityState        = 0x2404  ; //ID 2404h
const unsigned short QueryWrenchEffort         = 0x2405  ; //ID 2405h
const unsigned short QueryGlobalVector         = 0x2407  ; //ID 2407h
const unsigned short QueryLocalVector          = 0x2408  ; //ID 2408h
const unsigned short QueryTravelSpeed          = 0x240A  ; //ID 240Ah
const unsigned short QueryGlobalWaypoint       = 0x240C  ; //ID 240Ch
const unsigned short QueryLocalWaypoint        = 0x240D  ; //ID 240Dh
const unsigned short QueryGlobalPathSegment    = 0x240F  ; //ID 240Fh
const unsigned short QueryLocalPathSegment     = 0x2410  ; //ID 2410h
const unsigned short QueryGeomagneticProperty  = 0x2412  ; //ID 2412h
const unsigned short QueryVelocityCommand      = 0x2415  ; //ID 2415h
const unsigned short QueryAccelerationLimit    = 0x2416  ; //ID 2416h
const unsigned short QueryAccelerationState    = 0x2417  ; //ID 2417h
const unsigned short QueryElement              = 0x241A  ; //ID 241Ah
const unsigned short QueryElementList          = 0x241B  ; //ID 241Bh
const unsigned short QueryElementCount         = 0x241C  ; //ID 241Ch
const unsigned short QueryActiveElement        = 0x241E  ; //ID 241Eh
// InformClass
const unsigned short ReportGlobalPose          = 0x4402  ; //ID 4402h
const unsigned short ReportLocalPose           = 0x4403  ; //ID 4403h
const unsigned short ReportVelocityState       = 0x4404  ; //ID 4404h
const unsigned short ReportWrenchEffort        = 0x4405  ; //ID 4405h
const unsigned short ReportGlobalVector        = 0x4407  ; //ID 4407h
const unsigned short ReportLocalVector         = 0x4408  ; //ID 4408h
const unsigned short ReportTravelSpeed         = 0x440A  ; //ID 440Ah
const unsigned short ReportGlobalWaypoint      = 0x440C  ; //ID 440Ch
const unsigned short ReportLocalWaypoint       = 0x440D  ; //ID 440Dh
const unsigned short ReportGlobalPathSegment   = 0x440F  ; //ID 440Fh
const unsigned short ReportLocalPathSegment    = 0x4410  ; //ID 4410h
const unsigned short ReportGeomagneticProperty = 0x4412  ; //ID 4412h
const unsigned short ReportVelocityCommand     = 0x4415  ; //ID 4415h
const unsigned short ReportAccelerationLimit   = 0x4416  ; //ID 4416h
const unsigned short ReportAccelerationState   = 0x4417  ; //ID 4417h
const unsigned short ReportElement             = 0x441A  ; //ID 441Ah
const unsigned short ReportElementList         = 0x441B  ; //ID 441Bh
const unsigned short ReportElementCount        = 0x441C  ; //ID 441Ch
const unsigned short ReportActiveElement       = 0x441E  ; //ID 441Eh
#endif
