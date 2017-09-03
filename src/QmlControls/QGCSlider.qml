/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

import QtQuick                  2.3
import QtQuick.Controls         1.2
import QtQuick.Controls.Styles  1.4
import QtQuick.Controls.Private 1.0

import QGroundControl.Palette       1.0
import QGroundControl.ScreenTools   1.0

Slider {
    id:             _root
    implicitHeight: ScreenTools.implicitSliderHeight

    // Value indicator starts display from center instead of min value
    property bool indicatorCentered: false

    QGCPalette { id: qgcPal; colorGroupEnabled: enabled }

    style: SliderStyle {
        groove: Item {
            anchors.verticalCenter: parent.verticalCenter
            implicitWidth:  Math.round(ScreenTools.defaultFontPixelHeight * 4.5)
            implicitHeight: Math.round(ScreenTools.defaultFontPixelHeight * 0.3)

            Rectangle {
                radius:         height / 2
                anchors.fill:   parent
                color:          qgcPal.button
                border.width:   1
                border.color:   qgcPal.buttonText
            }

            Item {
                clip:   true
                x:      _root.indicatorCentered ? indicatorCenteredIndicatorStart : 0
                width:  _root.indicatorCentered ? centerIndicatorWidth : styleData.handlePosition
                height: parent.height

                property real indicatorCenteredIndicatorStart:   Math.min(styleData.handlePosition, parent.width / 2)
                property real indicatorCenteredIndicatorStop:    Math.max(styleData.handlePosition, parent.width / 2)
                property real centerIndicatorWidth:     indicatorCenteredIndicatorStop - indicatorCenteredIndicatorStart

                Rectangle {
                    anchors.fill:   parent
                    color:          qgcPal.colorBlue
                    border.color:   Qt.darker(color, 1.2)
                    radius:         height/2
                }
            }
        }

        handle: Rectangle {
            anchors.centerIn: parent
            color:          qgcPal.button
            border.color:   qgcPal.buttonText
            border.width:   1
            implicitWidth:  _radius * 2
            implicitHeight: _radius * 2
            radius:         _radius

            property real _radius: Math.round(ScreenTools.defaultFontPixelHeight * 0.75)
        }
    }
}