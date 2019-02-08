package org.waltonrobotics.util;

import org.waltonrobotics.controller.RobotPair;

public interface SetSpeeds {

  void setSpeeds(double left, double right);

  RobotPair getWheelPositions();

}
