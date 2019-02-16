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
    serialPort.setReadBufferSize(18);
  }

  public CameraData getCameraData() {
    return cameraData;
  }

  @Override
  public synchronized void run() {
    String data = serialPort.readString().trim();

//    System.out.println(data.matches("(F)|(\\d{2,})"));
    System.out.println(data);


//    if (data.length() > 17) {
    if (data.matches("^[xX]\\d{3}[yY]\\d{3}[zZ]\\d{3}[aA]\\d{3}N\\d+$")) {
      int x = Integer.parseUnsignedInt(data.substring(1, 4)) *
          (data.charAt(0) == 'X' ? 1 : -1);
      int y = Integer.parseUnsignedInt(data.substring(5, 8)) *
          (data.charAt(4) == 'Y' ? 1 : -1);
      int z = Integer.parseUnsignedInt(data.substring(9, 12)) *
          (data.charAt(8) == 'Z' ? 1 : -1);
      int angle = Integer.parseUnsignedInt(data.substring(13, 16)) *
          (data.charAt(12) == 'A' ? 1 : -1);
      int numberOfTargets = Integer.parseUnsignedInt(data.substring(17));
      double t = Timer.getFPGATimestamp() - 0.052;

      this.cameraData = new CameraData(x / 100.0, y / 100.0, z, angle, numberOfTargets, t);
    } else if (data.matches("^FN\\d+$")) {
      int numberOfTargets = Integer.parseUnsignedInt(data.substring(2));
      this.cameraData = new CameraData(numberOfTargets);
    } else {
      this.cameraData = new CameraData();
    }
  }

  public void startCollecting() {
    serialPort.writeString("S");
  }

  public void endCollecting() {
    serialPort.writeString("E");
  }
}
