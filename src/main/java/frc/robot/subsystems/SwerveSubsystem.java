
package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;
  public SwerveSubsystem() {

      boolean blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
          Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                            Meter.of(4)),
                                                          Rotation2d.fromDegrees(0))
                                            : new Pose2d(new Translation2d(Meter.of(16),
                                                                            Meter.of(4)),
                                                          Rotation2d.fromDegrees(180));
          SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maximumSpeed, startingPose);

      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
      swerveDrive.setHeadingCorrection(false);
      swerveDrive.setCosineCompensator(true); // DISABLE IF YOU ARE IN SIMULATION.
      // swerveDrive.setAngularVelocityCompensation(true,
      //   true,
      //   0.1);
    
    }

  @Override
  public void periodic() {
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
}
