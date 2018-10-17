package frc.team2974.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.Config.Path;
import frc.team2974.robot.subsystems.Drivetrain;
import org.waltonrobotics.MotionLogger;
import org.waltonrobotics.controller.Pose;
import org.waltonrobotics.motion.Line;
import org.waltonrobotics.motion.PointTurn;
import org.waltonrobotics.motion.Spline;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the IterativeRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the manifest file in the resource directory.
 */
public class Robot extends IterativeRobot {

  public static Drivetrain drivetrain;
  public static MotionLogger motionLogger;

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    motionLogger = new MotionLogger("/home/lvuser/");
    drivetrain = new Drivetrain(motionLogger);

    updateSmartDashboard();
  }

  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
    motionLogger.writeMotionDataCSV();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();
  }

  @Override
  public void autonomousInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.startControllerMotion(Pose.ZERO);
    motionLogger.initialize();

    Line starightLine = new Line(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        1);

    PointTurn pointTurn = new PointTurn(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, drivetrain.getActualPosition(), 180);

    Spline spline = new Spline(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        Pose.ZERO);

    drivetrain.addControllerMotions(starightLine, pointTurn, spline);

  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();
  }

  @Override
  public void teleopInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    updateSmartDashboard();
  }

  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * Put things in here you want to update for SmartDashboard.
   */
  private void updateSmartDashboard() {
    SmartDashboard.putString("Robot position", String.valueOf(drivetrain.getActualPosition()));

  }

}
