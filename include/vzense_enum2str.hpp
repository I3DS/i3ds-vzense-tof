///////////////////////////////////////////////////////////////////////////\file
///
///   Copyright 2022 SINTEF AS
///
///   This Source Code Form is subject to the terms of the Mozilla
///   Public License, v. 2.0. If a copy of the MPL was not distributed
///   with this file, You can obtain one at https://mozilla.org/MPL/2.0/
///
////////////////////////////////////////////////////////////////////////////////

#ifndef __VZENSE_ENUM2STR_HPP
#define __VZENSE_ENUM2STR_HPP

#include "Vzense_api2.h"

inline const char* returnStatus2string(PsReturnStatus status) {
  switch (status) {
    case PsRetOK:
      return "OK";
    case PsRetNoDeviceConnected:
      return "NoDeviceConnected";
    case PsRetInvalidDeviceIndex:
      return "InvalidDeviceIndex";
    case PsRetDevicePointerIsNull:
      return "DevicePointerIsNull";
    case PsRetInvalidFrameType:
      return "InvalidFrameType";
    case PsRetFramePointerIsNull:
      return "FramePointerIsNull";
    case PsRetNoPropertyValueGet:
      return "NoPropertyValueGet";
    case PsRetNoPropertyValueSet:
      return "NoPropertyValueSet";
    case PsRetPropertyPointerIsNull:
      return "PropertyPointerIsNull";
    case PsRetPropertySizeNotEnough:
      return "PropertySizeNotEnough";
    case PsRetInvalidDepthRange:
      return "InvalidDepthRange";
    case PsRetReadNextFrameTimeOut:
      return "ReadNextFrameTimeOut";
    case PsRetInputPointerIsNull:
      return "InputPointerIsNull";
    case PsRetCameraNotOpened:
      return "CameraNotOpened";
    case PsRetInvalidCameraType:
      return "InvalidCameraType";
    case PsRetInvalidParams:
      return "InvalidParams";
    case PsRetCurrentVersionNotSupport:
      return "CurrentVersionNotSupport";
    case PsRetUpgradeImgError:
      return "UpgradeImgError";
    case PsRetUpgradeImgPathTooLong:
      return "UpgradeImgPathTooLong";
    case PsRetUpgradeCallbackNotSet:
      return "UpgradeCallbackNotSet";
    case PsRetNoAdapterConnected:
      return "NoAdapterConnected";
    case PsRetReInitialized:
      return "ReInitialized";
    case PsRetNoInitialized:
      return "NoInitialized";
    case PsRetCameraOpened:
      return "CameraOpened";
    case PsRetCmdError:
      return "CmdError";
    case PsRetCmdSyncTimeOut:
      return "CmdSyncTimeOut";
    case PsRetOthers:
      return "Others";
    default:
      return "Undefined";
  }
}

inline const char* connectStatus2string(PsConnectStatus status) {
  switch (status) {
    case ConnectUNKNOWN:
      return "Unknown";
    case Unconnected:
      return "Unconnected";
    case Connected:
      return "Connected";
    case Opened:
      return "Opened";
    default:
      return "Undefined";
  }
}

inline const char* dataMode2str(PsDataMode mode) {
  switch (mode) {
    case PsDepthAndRGB_30:
      return "DepthAndRGB_30";
    case PsIRAndRGB_30:
      return "IRAndRGB_30";
    case PsDepthAndIR_30:
      return "PsDepthAndIR_30";
    case PsNoCCD_30:
      return "NoCCD_30";
    case PsDepthAndIR_15_RGB_30:
      return "DepthAndIR_15_RGB_30";
    case PsWDR_Depth:
      return "WDR_Depth";
    case PsWDR_IR:
      return "PsWDR_IR";
    case PsWDR_DepthAndIR:
      return "PsWDR_DepthAndIR";
    default:
      return "Undefined mode";
  }
}

#endif  //__VZENSE_ENUM2STR_HPP
