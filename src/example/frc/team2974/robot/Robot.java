package frc.team2974.robot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team2974.robot.Config.Path;
import frc.team2974.robot.subsystems.Drivetrain;
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

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit() {
    drivetrain = new Drivetrain();

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();
    drivetrain.getMotionLogger().writeMotionDataCSV();

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.startControllerMotion(Pose.ZERO);
    drivetrain.getMotionLogger().initialize();

    Line starightLine = new Line(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        1);

    PointTurn pointTurn = new PointTurn(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, drivetrain.getActualPosition(), 180);

    Spline spline = new Spline(Path.VELOCITY_MAX, Path.ACCELERATION_MAX, 0, 0, false, drivetrain.getActualPosition(),
        Pose.ZERO);

    drivetrain.addControllerMotions(starightLine, pointTurn, spline);

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    drivetrain.cancelControllerMotion();
    drivetrain.reset();

    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode
   */
  @Override
  public void testPeriodic() {
    updateSmartDashboard();
    Scheduler.getInstance().run();
  }

  /**
   * Put things in here you want to update for SmartDashboard.
   */
  private void updateSmartDashboard() {
    SmartDashboard.putString("Robot position", String.valueOf(drivetrain.getActualPosition()));

  }

}
