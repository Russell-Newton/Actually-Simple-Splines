package org.waltonrobotics.controller;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import java.util.TimerTask;

public class CameraReader extends TimerTask {

  private final SerialPort serialPort;
  private CameraData cameraData = new CameraData();

  public CameraReader() {
    serialPort = new SerialPort(115200, Port.kUSB);
    serialPort.enableTermination();
    serialPort.setReadBufferSize(1);
  }

  public CameraData getCameraData() {
    return cameraData;
  }

  @Override
  public synchronized void run() {
    String data = serialPort.readString();
    if (data.charAt(0) != 'F') {
      double x = Double.parseDouble(data.substring(1, 4)) *
          data.charAt(0) == 'X' ? 1 : -1;
      double y = Double.parseDouble(data.substring(5, 8)) *
          data.charAt(4) == 'Y' ? 1 : -1;
      double z = Double.parseDouble(data.substring(9, 11)) *
          data.charAt(8) == 'Z' ? 1 : -1;
      double angle = Double.parseDouble(data.substring(12, 15)) *
          data.charAt(11) == 'A' ? 1 : -1;
      int numberOfTargets = Integer.parseInt(data.substring(16));
      double t = Timer.getFPGATimestamp() - 0.052;
      this.cameraData = new CameraData(x, y, z, angle, numberOfTargets, t);
    } else {
      int numberOfTargets = Integer.parseInt(data.substring(2));
      this.cameraData = new CameraData(numberOfTargets);
    }
  }

  public void startCollecting() {
    serialPort.writeString("S");
  }

  public void endCollecting() {
    serialPort.writeString("E");
  }
}
