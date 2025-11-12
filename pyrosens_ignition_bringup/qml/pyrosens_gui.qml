// This file is loaded by gz-gui when the plugin is active.
// The C++ plugin instance is exposed as "PyroSENSGui"

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
  id: root
  width: 360
  implicitHeight: col.implicitHeight

  ColumnLayout {
    id: col
    anchors.fill: parent
    spacing: 10
    anchors.margins: 10

    GroupBox {
      title: "Mission Controls"
      Layout.fillWidth: true
      RowLayout {
        spacing: 8
        Button { text: "Start";   onClicked: PyroSENSGui.startMission() }
        Button { text: "Resume";  onClicked: PyroSENSGui.resumeMission() }
        Button { text: "Abort";   onClicked: PyroSENSGui.abortMission() }
        Button { text: "E-Stop";  onClicked: PyroSENSGui.estopMission() }
      }
    }

    GroupBox {
      title: "Mission Status"
      Layout.fillWidth: true
      Label {
        text: "Status: " + PyroSENSGui.missionStatus
        wrapMode: Text.WordWrap
      }
    }

    GroupBox {
      title: "Goals (sequential)"
      Layout.fillWidth: true
      Layout.fillHeight: true
      Layout.minimumHeight: 220
      ColumnLayout {
        spacing: 6
        Layout.fillWidth: true
        Layout.fillHeight: true
        Label {
          visible: PyroSENSGui.goals.length === 0
          text: "(no goals loaded)"
          wrapMode: Text.WordWrap
          Layout.fillWidth: true
          Layout.alignment: Qt.AlignTop
        }
        Repeater {
          model: PyroSENSGui.goals
          delegate: Frame {
            Layout.fillWidth: true
            Layout.preferredHeight: Math.max(goalLabel.implicitHeight + 12, 36)
            padding: 6
            background: Rectangle {
              color: index % 2 === 0 ? "#f5f5f5" : "#ffffff"
              radius: 2
            }
            Label {
              id: goalLabel
              text: (index + 1) + ". " + modelData
              wrapMode: Text.WordWrap
              Layout.fillWidth: true
              horizontalAlignment: Text.AlignLeft
              verticalAlignment: Text.AlignVCenter
            }
          }
        }
      }
    }

    GroupBox {
      title: "Telemetry"
      Layout.fillWidth: true
      ColumnLayout {
        Label { text: PyroSENSGui.windText; wrapMode: Text.WordWrap }
      }
    }

    Rectangle { height: 6; color: "transparent"; Layout.fillWidth: true }
  }
}
