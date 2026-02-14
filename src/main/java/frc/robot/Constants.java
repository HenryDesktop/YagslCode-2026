
package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  
  public static class SwerveConstants {
    public static final double maximumSpeed = Units.feetToMeters(7);
    public static final double deadband = 0.1;

  }
}
