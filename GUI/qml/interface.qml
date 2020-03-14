import QtQuick 2.6
import QtQuick.Controls 2.14
import QtQuick.Layouts 1.14
import QtQuick.Window 2.14

ApplicationWindow {
    id: mainWindow
    visible: true
    height: 550
    width: 850

    Timer {
        interval: 1500
        running: true
        repeat: true
        onTriggered: {
            sensor.refresh()
        }
    }

    ColumnLayout {
        id: mainLayout
        anchors.fill: parent
        Layout.margins: 25
        spacing: 45

        Text {
            id: title
            Layout.alignment: Qt.AlignHCenter
            Layout.topMargin: 8
            text: "Wheather station"
            font.weight: Font.DemiBold
            font.pointSize: 20 
        }

        RowLayout {
            id: temperature_banner
            Layout.topMargin: 15
            spacing: 35
            Image {
                Layout.leftMargin: 25
                source: "../img/temperature.png"
                sourceSize.width: 65
                sourceSize.height: 65
            }
            ProgressBar {
                implicitWidth: mainWindow.width * 0.65
                from: -20
                to: 50
                value: sensor.temperature
            }
            Text {
                Layout.rightMargin: 25
                text: sensor.temperature + " °C"
                font.weight: Font.DemiBold
                font.pointSize: 20
                color: "darkgrey"
            }
        }

        RowLayout {
            id: pressure_banner
            Layout.topMargin: 15
            spacing: 35
            Image {
                Layout.leftMargin: 25
                source: "../img/pressure.png"
                sourceSize.width: 65
                sourceSize.height: 65
            }
            ProgressBar {
                implicitWidth: mainWindow.width * 0.65
                from: 700
                to: 1300
                value: sensor.pressure
            }
            Text {
                Layout.rightMargin: 25
                text: sensor.pressure + " hPa"
                font.weight: Font.DemiBold
                font.pointSize: 15
                color: "darkgrey"
            }
        }

        RowLayout {
            id: humidity_banner
            Layout.topMargin: 15
            spacing: 35
            Image {
                Layout.leftMargin: 25
                source: "../img/humidity.png"
                sourceSize.width: 65
                sourceSize.height: 65
            }
            ProgressBar {
                implicitWidth: mainWindow.width * 0.65
                from: 0
                to: 100
                value: sensor.humidity
            }
            Text {
                Layout.rightMargin: 25
                text: sensor.humidity + " %"
                font.weight: Font.DemiBold
                font.pointSize: 20
                color: "darkgrey"
            }
        }
        ColumnLayout {}
    }
}