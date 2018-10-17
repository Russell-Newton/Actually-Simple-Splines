package frc.team2974.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Wrapper class for the logitech gamepad
 */
public class Gamepad extends Joystick {

  /**
   * @param port the port of the controller
   */
  public Gamepad(int port) {
    super(port);
  }

  /**
   * forward is -1 and backward is 1
   *
   * @return the leftJoystick thumb stick y value between -1 and 1
   */
  public double getLeftY() {
    return getRawAxis(1);
  }

  /**
   * motorLeft is 1 motorRight is -1
   *
   * @return the motorLeft thumb stick x value between -1 and 1
   */
  public double getLeftX() {
    return getRawAxis(0);
  }

  /**
   * leftJoystick is 1 motorRight is -1
   *
   * @return the motorRight thumb stick x value between -1 and 1
   */
  public double getRightX() {
    return getRawAxis(2);
  }

  /**
   * forward is -1 and backward is 1
   *
   * @return the motorRight thumb stick y value between -1 and 1
   */
  public double getRightY() {
    return getRawAxis(3);
  }

  /**
   * @param index the index of the button
   * @return true if button pressed false if not pressed
   */
  public boolean getButton(int index) {
    return getRawButton(index);
  }

  /**
   * @param b the button to get
   */
  public boolean getButton(Button b) {
    return b.getPressed(this);
  }

  public boolean getPOVButton(int angle) {
    return getPOV() == angle;
  }

  /**
   * @param p the POV to get based on compass directions <p> N,S,E,W,NE,NW,SE,SW, or CENTER
   * @return true if the POV button is pressed false if not
   */
  public boolean getPOVButton(POV p) {
    return p.getPressed(this);
  }

  /**
   * d-pad buttons enum
   */
  public enum POV {
    N(0), S(180), E(90), W(270), NE(45), SE(135), NW(315), SW(225), CENTER(0);

    private final int angle;

    POV(int angle) {
      this.angle = angle;
    }

    public int angle() {
      return angle;
    }

    public boolean getPressed(Gamepad g) {
      return g.getPOV() == angle;
    }
  }

  /**
   * non d-pad buttons enum
   */
  public enum Button {
    _1(1), _2(2), _3(3), _4(4),
    LEFT_BUMPER(5), RIGHT_BUMPER(6),
    LEFT_TRIGGER(7), RIGHT_TRIGGER(8),
    _9(9), _10(10),
    LEFT_STICK_BUTTON(11), RIGHT_STICK_BUTTON(12);

    private final int index;

    Button(int index) {
      this.index = index;
    }

    public int index() {
      return index;
    }

    public boolean getPressed(Gamepad g) {
      return g.getRawButton(index);
    }
  }
}
