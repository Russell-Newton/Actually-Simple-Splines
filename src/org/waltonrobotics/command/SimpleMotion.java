package org.waltonrobotics.command;

import edu.wpi.first.wpilibj.command.Command;
import org.waltonrobotics.AbstractDrivetrain;
import org.waltonrobotics.controller.Path;

public class SimpleMotion extends Command {

	private static AbstractDrivetrain drivetrain;
	private final Path path;

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

	public Path getPath() {
		return path;
	}

	protected void initialize() {
		drivetrain.addControllerMotions(path);
	}

	protected boolean isFinished() {
		return path.isFinished();
	}

	protected void interrupted() {
		this.end();
	}

	protected void end() {
		System.out.println("turn done");
		drivetrain.setSpeeds(0, 0);
	}
}
