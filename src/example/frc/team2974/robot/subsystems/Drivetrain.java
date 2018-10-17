package frc.team2974.robot.subsystems;

import static frc.team2974.robot.Config.Robot.COMPETITION;
import static frc.team2974.robot.RobotMap.encoderLeft;
import static frc.team2974.robot.RobotMap.encoderRight;
import static frc.team2974.robot.RobotMap.motorLeft;
import static frc.team2974.robot.RobotMap.motorRight;

import edu.wpi.first.wpilibj.Timer;
import frc.team2974.robot.Config.MotionConstants;
import frc.team2974.robot.Config.Path;
import frc.team2974.robot.command.teleop.DriveCommand;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.MotionLogger;
import org.waltonrobotics.controller.RobotPair;

/**
 *
 */
public class Drivetrain extends AbstractDrivetrain {

  public Drivetrain(MotionLogger motionLogger) {
    super(motionLogger);
    motorRight.setInverted(true);

    setEncoderDistancePerPulse();
  }

  @Override
  public RobotPair getWheelPositions() {
    return new RobotPair(encoderLeft.getDistance(), encoderRight.getDistance(),
        Timer.getFPGATimestamp());
  }

  @Override
  public double getRobotWidth() {
    return COMPETITION.getRobotWidth(); // in meters;
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveCommand());
  }

  @Override
  public void reset() {
    System.out.println("Reset Drivetrain");
    encoderLeft.reset();
    encoderRight.reset();
  }

  @Override
  public void setEncoderDistancePerPulse() {
    double distancePerPulse = COMPETITION.getDistancePerPulse();

    encoderLeft.setDistancePerPulse(distancePerPulse);
    encoderRight.setDistancePerPulse(distancePerPulse);
    encoderRight.setReverseDirection(true);
    motorRight.setInverted(true);

  }

  @Override
  public void setSpeeds(double leftPower, double rightPower) {
    motorRight.set(-rightPower);
    motorLeft.set(-leftPower);
  }

  @Override
  public double getKV() {
    return MotionConstants.KV;
  }

  @Override
  public double getKAcc() {
    return MotionConstants.KAcc;
  }

  @Override
  public double getKK() {
    return MotionConstants.KK;
  }

  @Override
  public double getKS() {
    return MotionConstants.KS;
  }

  @Override
  public double getKAng() {
    return MotionConstants.KAng;
  }

  @Override
  public double getKL() {
    return MotionConstants.KL;
  }

  @Override
  public double getILag() {
    return MotionConstants.IL;
  }

  @Override
  public double getIAng() {
    return MotionConstants.IAng;
  }

  @Override
  public double getMaxVelocity() {
    return Path.VELOCITY_MAX;
  }

  @Override
  public double getMaxAcceleration() {
    return Path.ACCELERATION_MAX;
  }

}
