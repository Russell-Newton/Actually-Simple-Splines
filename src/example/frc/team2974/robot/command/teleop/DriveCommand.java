package frc.team2974.robot.command.teleop;

import static frc.team2974.robot.OI.leftJoystick;
import static frc.team2974.robot.OI.rightJoystick;
import static frc.team2974.robot.Robot.drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveCommand extends Command {

  public DriveCommand() {
    requires(drivetrain);
  }

  public double getLeftThrottle() {
    if (Math.abs(leftJoystick.getY()) < 0.3) {
      return 0;
    }
    return leftJoystick.getY();
  }

  public double getRightThrottle() {
    if (Math.abs(rightJoystick.getY()) < 0.3) {
      return 0;
    }
    return rightJoystick.getY();
  }

  public double getTurn() {
    if (Math.abs(rightJoystick.getX()) < 0.1) {
      return 0;
    }
    return rightJoystick.getX();
  }

  private void cheesyDrive() {
    double throttle = (-getLeftThrottle() + 1) / 2;
    double forward = -getRightThrottle();
    double turn = -getTurn();

    drivetrain.setSpeeds(throttle * (forward - turn), throttle * (forward + turn));
  }

  private void tankDrive() {
    double leftPower = -getLeftThrottle();
    double rightPower = -getRightThrottle();

    drivetrain.setSpeeds(leftPower, rightPower);
  }
  // Called just before this Command runs the first time

  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run

  @Override
  protected void execute() {
    tankDrive();
  }

  // Make this return true when this Command no longer needs to run execute()
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  protected void end() {
    drivetrain.setSpeeds(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  protected void interrupted() {
    end();
  }
}
