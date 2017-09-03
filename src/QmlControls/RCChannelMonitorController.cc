/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


#include "RCChannelMonitorController.h"

RCChannelMonitorController::RCChannelMonitorController(void)
    : _chanCount(0)
{
    connect(_vehicle, &Vehicle::rcChannelsChanged, this, &RCChannelMonitorController::_rcChannelsChanged);
}

void RCChannelMonitorController::_rcChannelsChanged(int channelCount, int pwmValues[Vehicle::cMaxRcChannels])
{
    int maxChannel = std::min(channelCount, _chanMax());

    for (int channel=0; channel<maxChannel; channel++) {
        int channelValue = pwmValues[channel];

        if (_chanCount != channelCount) {
            _chanCount = channelCount;
            emit channelCountChanged(_chanCount);
        }

        if (channelValue != -1) {
            emit channelRCValueChanged(channel, channelValue);
        }
    }
}

int RCChannelMonitorController::_chanMax(void) const
{
    return _vehicle->firmwareType() == MAV_AUTOPILOT_PX4 ? _chanMaxPX4 : _chanMaxAPM;
}
