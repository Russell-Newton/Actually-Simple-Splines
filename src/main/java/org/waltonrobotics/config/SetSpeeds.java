package org.waltonrobotics.config;

import org.waltonrobotics.metadata.RobotPair;

public interface SetSpeeds {

  void setSpeeds(double left, double right);

  RobotPair getWheelPositions();

}
