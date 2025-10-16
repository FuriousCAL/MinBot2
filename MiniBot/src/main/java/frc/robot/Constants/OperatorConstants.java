package frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class OperatorConstants {
  // Joystick port for the driver controller
  public static final int kDriverControllerPort = 0;
  
  // Joystick deadband
  public static final double kDriveDeadband = 0.05;
  
  // Joystick axis indices
  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;
  
  // Joystick button indices
  public static final int kZeroGyroButton = 7; // Start/Options button
  public static final int kResetOdometryButton = 8; // Back/View button
}
