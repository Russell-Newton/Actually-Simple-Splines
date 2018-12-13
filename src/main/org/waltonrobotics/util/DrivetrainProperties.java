package org.waltonrobotics.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;
import org.waltonrobotics.AbstractDrivetrain;

public class DrivetrainProperties extends Properties {

  private static final File CONFIG_FILE = new File("config.properties");
  private final AbstractDrivetrain drivetrain;

  public DrivetrainProperties(AbstractDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  /**
   * Saves the drivetrain properties to a config.properties file
   */
  public final void saveToFile() {
    try
        (FileWriter writer = new FileWriter(CONFIG_FILE)) {
      store(writer, "drive train settings");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }


  public void saveProperties() {
    setProperty("robotWidth", String.valueOf(drivetrain.getRobotWidth()));
    setProperty("maxVelocity", String.valueOf(drivetrain.getMaxVelocity()));
    setProperty("maxAcceleration", String.valueOf(drivetrain.getMaxAcceleration()));

    setProperty("kV", String.valueOf(drivetrain.getKV()));
    setProperty("kAcc", String.valueOf(drivetrain.getKAcc()));
    setProperty("kK", String.valueOf(drivetrain.getKK()));
    setProperty("kS", String.valueOf(drivetrain.getKS()));
    setProperty("kAng", String.valueOf(drivetrain.getKAng()));
    setProperty("kL", String.valueOf(drivetrain.getKL()));
    setProperty("iLag", String.valueOf(drivetrain.getILag()));
    setProperty("iAng", String.valueOf(drivetrain.getIAng()));
  }

  public final void savePIDConstants(HashMap<String, String> pidConstants) {
    setProperty("kV", pidConstants.get("kV"));
    setProperty("kAcc", pidConstants.get("kAcc"));
    setProperty("kK", pidConstants.get("kK"));
    setProperty("kS", pidConstants.get("kS"));
    setProperty("kAng", pidConstants.get("kAng"));
    setProperty("kL", pidConstants.get("kL"));
    setProperty("iLag", pidConstants.get("iLag"));
    setProperty("iAng", pidConstants.get("iAng"));

    saveToFile();
  }
}
