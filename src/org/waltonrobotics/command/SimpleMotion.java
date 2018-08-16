package org.waltonrobotics.command;

import edu.wpi.first.wpilibj.command.Command;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.motion.Path;

public class SimpleMotion extends Command {

    private static AbstractDrivetrain drivetrain;
    protected Path path;

    public SimpleMotion(Path path) {
        this.path = path;

        requires(drivetrain);
    }

    public static AbstractDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public static void setDrivetrain(AbstractDrivetrain drivetrain) {
        SimpleMotion.drivetrain = drivetrain;
    }

    @Override
    public String toString() {
        return "SimpleMotion{" +
                "path=" + path +
                '}';
    }

    public Path getPath() {
        return path;
    }

    @Override
    protected void initialize() {
        drivetrain.addControllerMotions(path);
    }

    @Override
    protected boolean isFinished() {
        return path.isFinished();
    }

    @Override
    protected void interrupted() {
        this.end();
    }

    @Override
    protected void end() {
        System.out.println("turn done");
        drivetrain.setSpeeds(0, 0);
    }
}
