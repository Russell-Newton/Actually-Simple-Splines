package org.waltonrobotics.util;

public interface EncoderConfig {

  double getDistancePerPulse();

  int getChannell1();

  int getChannell2();

  boolean isInverted();
}
