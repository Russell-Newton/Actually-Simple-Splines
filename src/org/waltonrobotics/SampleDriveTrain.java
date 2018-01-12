package org.waltonrobotics;

import org.waltonrobotics.controller.RobotPair;

/**
 * This is a sample drive train. If you want to use our motions for autonomous,
 * everything here will need to included in your drivetrain subsystem. This
 * includes the left and right encoders, the MotionController, and every
 * inherited method from the DriveTrainInterface
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class SampleDrivetrain extends AbstractDrivetrain {

	// private Encoder rightEncoder = RobotMap.rightEncoder;
	// private Encoder leftEncoder = RobotMap.leftEncoder;
	// private CANTalon rightMotor = RobotMap.rightMotor;
	// private CANTalon leftMotor = RobotMap.leftMotor;

	public SampleDrivetrain() {
	}

	@Override
	public RobotPair getWheelPositions() {
		// return new RobotPair(leftEncoder.getDistance(), rightEncoder.getDistance());
		return null;
	}

	@Override
	public void reset() {
		// leftEncoder.reset();
		// rightEncoder.reset();
	}

	@Override
	public void setSpeeds(double leftSpeed, double rightSpeed) {
		// leftMotor.setSpeed(leftSpeed);
		// rightMotor.setSpeed(rightSpeed);
	}

	@Override
	public void setEncoderDistancePerPulse() {
		// leftEncoder.setDistancePerPulse(0.025);
		// rightEncoder.setDistancePerPulse(0.025);

	}

	@Override
	protected void initDefaultCommand() {

	}
}
