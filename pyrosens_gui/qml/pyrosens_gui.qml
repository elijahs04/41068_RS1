// SPDX-License-Identifier: MIT
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

// This file is loaded by gz-gui when the plugin is active.
// The C++ plugin instance is exposed as "PyroSENSGui"

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
      ListView {
        id: goalsView
        Layout.fillWidth: true
        Layout.fillHeight: true
        model: PyroSENSGui.goals
        clip: true
        delegate: Rectangle {
          width: goalsView.width
          height: Math.max(28, textItem.implicitHeight + 10)
          color: index % 2 === 0 ? "#f5f5f5" : "#ffffff"
          Text {
            id: textItem
            anchors.margins: 6
            anchors.fill: parent
            text: (index+1) + ". " + modelData
            wrapMode: Text.WordWrap
          }
        }
      }
    }

    GroupBox {
      title: "Telemetry"
      Layout.fillWidth: true
      ColumnLayout {
        Label { text: PyroSENSGui.windText; wrapMode: Text.WordWrap }
        // Extend here with more telemetry bindings
        // e.g., Label { text: "Battery: " + PyroSENSGui.batteryText }
      }
    }

    Rectangle { height: 6; color: "transparent"; Layout.fillWidth: true }
  }
}
