package org.waltonrobotics.controller;

import java.util.ListIterator;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.MotionLogger;

/**
 * Controls Path motions
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class MotionController {

	/**
	 * 1 Runs the calculations with a TimerTask
	 * 
	 * @author Russell Newton, WaltonRobotics
	 *
	 */
	private class MotionTask extends TimerTask {

		@Override
		public void run() {
			if (currentPath != null) {
				RobotPair wheelPositions = drivetrain.getWheelPositions();
				updateActualPosition(wheelPositions);
				findCurrentError();
				powers = calculateSpeeds(wheelPositions);
				drivetrain.setSpeeds(powers.getLeft(), powers.getRight());
				motionLogger.addMotionData(
						new MotionData(actualPosition, targetPathData.getCenterPose(), errorVector, powers));
			}
		}

	}

	private BlockingDeque<Path> paths = new LinkedBlockingDeque<Path>();
	private Timer controller;
	private boolean running;
	private int period;
	private Path currentPath = null;
	private PathData staticPathData;
	private final AbstractDrivetrain drivetrain;
	private final double kV;
	private final double kK;
	private final double kAcc;
	private final double kS;
	private final double kL;
	private final double kAng;
	private Pose actualPosition;
	private PathData targetPathData;
	private RobotPair previousLengths;
	private RobotPair startingWheelPositions;
	private ListIterator<PathData> pdIterator;
	private PathData pdPrevious;
	private PathData pdNext;
	private ErrorVector errorVector;
	private MotionLogger motionLogger;
	private RobotPair powers;

	/**
	 * @param drivetrain
	 *            - the drivetrain to use the AbstractDrivetrain methods from
	 * @param motionLogger
	 *            - the MotionLogger from the AbstractDrivetrain
	 */
	public MotionController(AbstractDrivetrain drivetrain, MotionLogger motionLogger) {
		running = false;

		this.motionLogger = motionLogger;

		controller = new Timer();
		this.period = 5;
		controller.schedule(new MotionTask(), 0L, (long) period);

		this.drivetrain = drivetrain;
		this.kV = drivetrain.getKV();
		this.kK = drivetrain.getKK();
		this.kAcc = drivetrain.getKAcc();
		this.kS = drivetrain.getKS();
		this.kL = drivetrain.getKL();
		this.kAng = drivetrain.getKAng();
	}

	/**
	 * Adds a path to the path queue
	 * 
	 * @param paths
	 *            - the paths to add to the queue
	 */
	public void addPaths(Path... paths) {
		for (Path path : paths) {
			this.paths.addLast(path);
		}
	}

	/**
	 * Calculates the powers to send to the wheels
	 * 
	 * @return a RobotPair with the powers and the time
	 */
	private RobotPair calculateSpeeds(RobotPair wheelPositions) {

		double leftPower = 0;
		double rightPower = 0;
		boolean enabled;
		double steerPowerXTE;
		double steerPowerAngle;
		double centerPowerLag;

		synchronized (this) {
			enabled = this.running;
		}

		if (enabled) {

			if (currentPath != null) {
				targetPathData = interpolate(wheelPositions);
			} else {
				targetPathData = staticPathData;
			}

			if (currentPath.isFinished) {
				currentPath = paths.pollFirst();
				if (currentPath != null) {
					startingWheelPositions = wheelPositions;
				} else {
					System.out.println("Done with motions! :)");
					return new RobotPair(0, 0, wheelPositions.getTime());
				}
				staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
						new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0);
				targetPathData = staticPathData;
				pdIterator = currentPath.getPathData().listIterator();
				pdPrevious = targetPathData = pdIterator.next();
				pdNext = pdIterator.next();
				return new RobotPair(0, 0, wheelPositions.getTime());
			}

			synchronized (this) {
				// feed forward
				leftPower += (kV * targetPathData.getLeftState().getVelocity()
						+ kK * Math.signum(targetPathData.getLeftState().getVelocity()))
						+ kAcc * targetPathData.getLeftState().getAcceleration();
				rightPower += (kV * targetPathData.getRightState().getVelocity()
						+ kK * Math.signum(targetPathData.getRightState().getVelocity()))
						+ kAcc * targetPathData.getRightState().getAcceleration();
				// feed back
				steerPowerXTE = kS * errorVector.getXTrack();
				steerPowerAngle = kAng * errorVector.getAngle();
				centerPowerLag = kL * errorVector.getLag();
			}
			double centerPower = (leftPower + rightPower) / 2 + centerPowerLag;
			double steerPower = Math.max(-1,
					Math.min(1, (rightPower - leftPower) / 2 + steerPowerXTE + steerPowerAngle));
			centerPower = Math.max(-1 + Math.abs(steerPower), Math.min(1 - Math.abs(steerPower), centerPower));
			return new RobotPair(centerPower - steerPower, centerPower + steerPower, wheelPositions.getTime());
		}
		return new RobotPair(0, 0, wheelPositions.getTime());
	}

	/**
	 * Finds the target x, y, angle, velocityLeft, and velocityRight
	 * 
	 * @param wheelPositions
	 * @return a new MotionData with the interpolated data
	 */
	private PathData interpolate(RobotPair wheelPositions) {
		double currentTime = wheelPositions.getTime() - startingWheelPositions.getTime();
		while (currentTime > pdNext.getTime()) {
			if (pdIterator.hasNext()) {
				pdPrevious = pdNext;
				pdNext = pdIterator.next();
			} else {
				pdPrevious = pdNext;
				currentPath.isFinished = true;
				return pdNext;
			}
		}

		double timePrevious = pdPrevious.getTime();
		double timeNext = pdNext.getTime();
		double dTime = timeNext - timePrevious;
		double rctn = (timeNext - currentTime) / dTime; // Ratio of the current time to the next pose time
		double rltc = (currentTime - timePrevious) / dTime; // Ratio of the previous time to the current pose
															// time

		double lengthLeft = (pdPrevious.getLeftState().getLength()) * rctn + (pdNext.getLeftState().getLength()) * rltc;
		double lengthRight = (pdPrevious.getRightState().getLength()) * rctn
				+ (pdNext.getRightState().getLength()) * rltc;

		// Current pose is made from the weighted average of the x, y, and angle values
		double x = pdPrevious.getCenterPose().getX() * rctn + pdNext.getCenterPose().getX() * rltc;
		double y = pdPrevious.getCenterPose().getY() * rctn + pdNext.getCenterPose().getY() * rltc;
		double angle = pdPrevious.getCenterPose().getAngle() * rctn + pdNext.getCenterPose().getAngle() * rltc;

		State left = new State(lengthLeft, pdNext.getLeftState().getVelocity(),
				pdNext.getLeftState().getAcceleration());
		State right = new State(lengthRight, pdNext.getRightState().getVelocity(),
				pdNext.getRightState().getAcceleration());
		Pose centerPose = new Pose(x, y, angle);
		return new PathData(left, right, centerPose, currentTime);
	}

	/**
	 * Removes all queued motions
	 */
	public void clearMotions() {
		paths.clear();
	}

	/**
	 * Starts the queue of motions
	 */
	public void enableScheduler() {
		if (!running) {
			staticPathData = new PathData(new State(drivetrain.getWheelPositions().getLeft(), 0, 0),
					new State(drivetrain.getWheelPositions().getRight(), 0, 0), new Pose(0, 0, 0), 0);
			Path newPath = paths.poll();
			if (newPath != null) {
				currentPath = newPath;
				running = true;
			}
			actualPosition = currentPath.getPathData().get(0).getCenterPose();
			previousLengths = startingWheelPositions = drivetrain.getWheelPositions();
			pdIterator = currentPath.getPathData().listIterator();
			pdPrevious = targetPathData = pdIterator.next();
			pdNext = pdIterator.next();
		}
	}

	/**
	 * 
	 * @return Whether or not the queue has ended
	 */
	public boolean isFinished() {
		return currentPath == null;
	}

	/**
	 * 
	 * @return Whether or not a motion is running
	 */
	public boolean isRunning() {
		return running;
	}

	/**
	 * Pauses the motions,
	 */
	public void stopScheduler() {
		running = false;
		currentPath = null;
		controller.cancel();
		drivetrain.setSpeeds(0, 0);
	}

	/**
	 * Updates where the robot thinks it is, based off of the encoder lengths
	 * 
	 * @param wheelPositions
	 */
	private void updateActualPosition(RobotPair wheelPositions) {
		double arcLeft = wheelPositions.getLeft() - previousLengths.getLeft();
		double arcRight = wheelPositions.getRight() - previousLengths.getRight();
		double dAngle = (arcRight - arcLeft) / currentPath.robotWidth;
		double arcCenter = (arcRight + arcLeft) / 2;
		double dX;
		double dY;
		if (Math.abs(dAngle) < 0.01) {
			dX = arcCenter * Math.cos(actualPosition.getAngle());
			dY = arcCenter * Math.sin(actualPosition.getAngle());
		} else {
			dX = arcCenter * ((Math.sin(dAngle) * Math.cos(actualPosition.getAngle()) / dAngle)
					- ((Math.cos(dAngle) - 1) * Math.sin(actualPosition.getAngle()) / dAngle));
			dY = arcCenter * ((Math.sin(dAngle) * Math.sin(actualPosition.getAngle()) / dAngle)
					- ((Math.cos(dAngle) - 1) * Math.cos(actualPosition.getAngle()) / dAngle));
		}

		actualPosition = actualPosition.offset(dX, dY, dAngle);
		previousLengths = wheelPositions;
	}

	/**
	 * Finds the current lag and cross track ErrorVector
	 */
	private void findCurrentError() {
		Pose targetPose = targetPathData.getCenterPose();
		Pose actualPose = actualPosition;
		double dX = targetPose.getX() - actualPose.getX();
		double dY = targetPose.getY() - actualPose.getY();
		double angle = targetPose.getAngle();
		// error in direction facing
		double lagError = dX * Math.cos(angle) + dY * Math.sin(angle);
		// error perpendicular to direction facing
		double crossTrackError = -dX * Math.sin(angle) + dY * Math.cos(angle);
		// the error of the current angle
		double angleError = targetPose.getAngle() - actualPose.getAngle();
		if (angleError > Math.PI) {
			angleError -= 2 * Math.PI;
		} else if (angleError < -Math.PI) {
			angleError += 2 * Math.PI;
		}
		errorVector = new ErrorVector(lagError, crossTrackError, angleError);
	}
}
