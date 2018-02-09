package org.waltonrobotics.controller;

import java.util.ListIterator;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.BlockingDeque;
import java.util.concurrent.LinkedBlockingDeque;

import org.waltonrobotics.AbstractDrivetrain;

/**
 * Sends power to the wheels
 * 
 * @author Russell Newton, Walton Robotics
 *
 */
public class MotionController {

	private class MotionTask extends TimerTask {

		@Override
		public void run() {
			if (currentPath != null) {
				RobotPair wheelPositions = drivetrain.getWheelPositions();
				calculateSpeeds(wheelPositions);
				updateActualPosition(wheelPositions);
				findCurrentError();
				System.out.printf("lag: %f xTrack: %f \n", errorVector[0], errorVector[1]);
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
	private final double kA;
	private final double kP;
	private Pose actualPosition;
	private PathData targetPathData;
	private RobotPair previousLengths;
	private RobotPair startingWheelPositions;
	private ListIterator<PathData> pdIterator;
	private PathData pdPrevious;
	private PathData pdNext;
	private double[] errorVector;

	/**
	 * @param drivetrain
	 *            - the drivetrain to use the AbstractDrivetrain methods from
	 */
	public MotionController(AbstractDrivetrain drivetrain) {
		running = false;

		controller = new Timer();
		this.period = 5;
		controller.schedule(new MotionTask(), 0L, (long) period);

		this.drivetrain = drivetrain;
		this.kV = drivetrain.getKV();
		this.kK = drivetrain.getKK();
		this.kA = drivetrain.getKA();
		this.kP = drivetrain.getKP();
		errorVector = new double[2];

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
	 */
	private void calculateSpeeds(RobotPair wheelPositions) {

		double leftPower = 0;
		double rightPower = 0;
		boolean enabled;

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
				}
				staticPathData = new PathData(new State(wheelPositions.getLeft(), 0, 0),
						new State(wheelPositions.getRight(), 0, 0), new Pose(0, 0, 0), 0);
				targetPathData = staticPathData;
				pdIterator = currentPath.getPathData().listIterator();
				pdPrevious = pdIterator.next();
				pdNext = pdIterator.next();
				return;
			}

			synchronized (this) {
				// feed forward
				leftPower += (kV * targetPathData.getLeftState().getVelocity() + kK)
						+ kA * targetPathData.getLeftState().getAcceleration();
				rightPower += (kV * targetPathData.getRightState().getVelocity() + kK)
						+ kA * targetPathData.getRightState().getAcceleration();
				// feed back
				leftPower += kP * (targetPathData.getLeftState().getLength() - wheelPositions.getLeft()
						- startingWheelPositions.getLeft());
				rightPower += kP * (targetPathData.getRightState().getLength() - wheelPositions.getRight()
						- startingWheelPositions.getRight());
			}

			leftPower = Math.max(-1, Math.min(1, leftPower));
			rightPower = Math.max(-1, Math.min(1, rightPower));

			drivetrain.setSpeeds(leftPower, rightPower);
		}
	}

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
			pdPrevious = pdIterator.next();
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
	 * @param width
	 *            - the robot width
	 * @param previous
	 *            - a RobotPair with the previous encoder lengths
	 * @param wheelPositions
	 *            - a RobotPair with the current encoder lengths
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

	private void findCurrentError() {
		Pose targetPose = targetPathData.getCenterPose();
		Pose actualPose = actualPosition;
		double dX = actualPose.getX() - targetPose.getX();
		double dY = actualPose.getY() - targetPose.getY();
		double angle = actualPose.getAngle();
		// error in direction facing
		double lagError = dX * Math.cos(angle) + dY * Math.sin(angle);
		// error perpendicular to direction facing
		double crossTrackError = -dX * Math.sin(angle) + dY * Math.cos(angle);
		errorVector = new double[] {lagError, crossTrackError};
	}
}
