package org.waltonrobotics.command;

import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Spline;

import java.util.Arrays;

/**
 *
 */
public class SimpleSpline extends SimpleMotion {

    public SimpleSpline(double maxVelocity, double maxAcceleration, Pose... knots) {
        this(maxVelocity, maxAcceleration, false, knots);
    }

    public SimpleSpline(double maxVelocity, double maxAcceleration, boolean isBackwards,
                        double startScale,
                        double endScale, double startVelocity,
                        double endVelocity, Pose... knots) {
        super(new Spline(
                maxVelocity,
                maxAcceleration,
                startVelocity,
                endVelocity,
                isBackwards,
                startScale,
                endScale,
                Arrays.asList(knots)));
    }

    public SimpleSpline(double maxVelocity, double maxAcceleration, boolean isBackwards,
                        Pose... knots) {
        this(maxVelocity, maxAcceleration, isBackwards, 1,
                1, 0, 0, knots);
    }

    public static SimpleSpline pathFromPosesWithAngle(boolean isBackwards, Pose... knots) {
        return pathFromPosesWithAngle(getDrivetrain().getMaxVelocity(),
                getDrivetrain().getMaxAcceleration(),
                isBackwards, knots);
    }

    /**
     * Creates a SimpleSpline where the first angle is the angle of the first point and the final
     * angle is the angle of the last point.
     *
     * @param isBackwards will the robot move forwards or backwards
     * @param knots       the points (with angle) to move through
     * @return a new SimpleSpline command
     */
    public static SimpleSpline pathFromPosesWithAngle(double maxVelocity, double maxAcceleration,
                                                      boolean isBackwards, Pose... knots) {
        return new SimpleSpline(maxVelocity, maxAcceleration, isBackwards, knots);
    }


    public static SimpleSpline pathFromPosesWithAngle(double maxVelocity, double maxAcceleration,
                                                      double startVelocity, double endVelocity, boolean isBackwards, Pose... knots) {
        return new SimpleSpline(maxVelocity, maxAcceleration, isBackwards, 1, 1, startVelocity,
                endVelocity, knots);
    }


    public static SimpleSpline pathFromPosesWithAngleAndScale(double maxVelocity,
                                                              double maxAcceleration,
                                                              boolean isBackwards, double startScale, double endScale, Pose... knots) {
        return new SimpleSpline(maxVelocity, maxAcceleration, isBackwards, startScale, endScale, 0, 0,
                knots);
    }

    public static SimpleSpline pathFromPosesWithAngleAndScale(
            boolean isBackwards, double startScale, double endScale, Pose... knots) {
        return pathFromPosesWithAngleAndScale(getDrivetrain().getMaxVelocity(),
                getDrivetrain().getMaxAcceleration(),
                isBackwards,
                startScale, endScale, knots);
    }

}
