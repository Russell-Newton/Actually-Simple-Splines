package frc.team2974.robot.subsystems;

import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.RobotPair;
import org.waltonrobotics.util.RobotConfig;

/**
 *
 */
public class Drivetrain extends AbstractDrivetrain {


  public Drivetrain(RobotConfig robotConfig) {
    super(robotConfig);
  }

  @Override
  public RobotPair getWheelPositions() {
    return null;
  }

  @Override
  public void reset() {

  }

  @Override
  public void setSpeeds(double leftPower, double rightPower) {

  }

  @Override
  public void setEncoderDistancePerPulse() {

  }

  @Override
  protected void initDefaultCommand() {

  }
}
