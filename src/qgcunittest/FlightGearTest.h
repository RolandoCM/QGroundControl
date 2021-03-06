/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


#ifndef FLIGHTGEARTEST_H
#define FLIGHTGEARTEST_H

#include "UnitTest.h"
#include "TCPLink.h"
#include "MultiSignalSpy.h"

/// @file
///     @brief FlightGear HIL Simulation unit tests
///
///     @author Don Gagne <don@thegagnes.com>

class FlightGearUnitTest : public UnitTest
{
    Q_OBJECT
    
public:
    FlightGearUnitTest(void);
    
private slots:
    void _parseUIArguments_test(void);
};

#endif
