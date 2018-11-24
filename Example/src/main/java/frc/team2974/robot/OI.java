package frc.team2974.robot;

import static frc.team2974.robot.Config.Input.GAMEPAD_PORT;
import static frc.team2974.robot.Config.Input.LEFT_JOYSTICK_PORT;
import static frc.team2974.robot.Config.Input.RIGHT_JOYSTICK_PORT;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
public final class OI {

  public static final Joystick leftJoystick;
  public static final Joystick rightJoystick;
  public static final Gamepad gamepad;

  static {
    leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
    rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);
    gamepad = new Gamepad(GAMEPAD_PORT);

  }

  private OI() {
  }
}
