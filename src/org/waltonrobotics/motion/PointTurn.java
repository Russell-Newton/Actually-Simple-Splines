package org.waltonrobotics.motion;

import java.util.LinkedList;

import org.waltonrobotics.controller.Path;
import org.waltonrobotics.controller.PathData;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.controller.State;

public class PointTurn extends Path {
	private final double startAngle;
	private final double endAngle;
	private final double turnAngle;
	private final double turnStep;
	private final double vRotationMax;
	private final double aRotationMax;
	private final LinkedList<PathData> pathData;

	public PointTurn(double vRotationMax, double aRotationMax, double startAngle, double endAngle) {
		super(vRotationMax, aRotationMax, false, new Pose[] { new Pose(0, 0, startAngle), new Pose(0, 0, endAngle) });
		this.startAngle = Math.toRadians(startAngle);
		this.endAngle = Math.toRadians(endAngle);
		this.turnAngle = this.endAngle - this.startAngle;
		this.vRotationMax = vRotationMax;
		this.aRotationMax = aRotationMax;
		turnStep = turnAngle/50;
		this.pathData = new LinkedList<>();
		generateData();
	}
	
	private void generateData() {
		pathData.add(getPathData(new PathData(new Pose(0, 0, startAngle))));
		for(int i = 2; i <= 50; i++) {
			pathData.add(getPathData(pathData.getLast()));
		}
	}
	
	private PathData getPathData(PathData previous) {

	    // estimate lengths each wheel will turn
	    double dlLeft = (-turnStep * getRobotWidth() / 2);
	    double dlRight = (turnStep * getRobotWidth() / 2);

	    // assuming one of the wheels will limit motion, calculate time this step will
	    // take
	    double dt = Math.max(Math.abs(dlLeft), Math.abs(dlRight)) / vRotationMax;
	    double a = 0.0;
	    
	 // assuming constant angular acceleration from/to zero angular speed
        double omega = Math.abs(dlRight - dlLeft) / dt / getRobotWidth();
        double thetaMidpoint = previous.getCenterPose().getAngle() + .5 * turnStep;

        double omegaAccel = Math.sqrt(aRotationMax
            * Math.abs(boundAngle(thetaMidpoint - startAngle)));
        double omegaDecel = Math.sqrt(aRotationMax
            * Math.abs(boundAngle(thetaMidpoint - endAngle)));
         System.out.println("OmegaAccel=" + omegaAccel);
//         System.out.println(omega);
        if (omegaAccel < omega && omegaAccel < omegaDecel) {
          dt = Math.abs(dlRight - dlLeft) / omegaAccel / getRobotWidth();
        }

         System.out.println("OmegaDecel=" + omegaDecel);
        if (omegaDecel < omega && omegaDecel < omegaAccel) {
          dt = Math.abs(dlRight - dlLeft) / omegaDecel / getRobotWidth();
        }
//        System.out.println(dt);
        
        State left = new State(previous.getLeftState().getLength() + dlLeft, dlLeft / dt, a);
        State right = new State(previous.getRightState().getLength() + dlRight, dlRight / dt, a);
        Pose center = new Pose(0, 0, previous.getCenterPose().getAngle() + turnStep);
        
	    return new PathData(left, right, center, previous.getTime() + dt);
	}
	
	public double boundAngle(double angle) {
		if(angle > Math.PI) {
			return angle - Math.PI;
		}
		if(angle < -Math.PI) {
			return angle + Math.PI;
		}
		return angle;
	}

	@Override
	public LinkedList<PathData> getPathData() {
		return pathData;
	}

}
