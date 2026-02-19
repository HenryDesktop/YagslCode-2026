
package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  public SwerveSubsystem() {

      // boolean blueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
      //     Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
      //                                                                       Meter.of(4)),
      //                                                     Rotation2d.fromDegrees(0))
      //                                       : new Pose2d(new Translation2d(Meter.of(16),
      //                                                                       Meter.of(4)),
      //                                                     Rotation2d.fromDegrees(180));
      //     SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.maximumSpeed, new Pose2d(new Translation2d(Meter.of(1),
                                                                                                                                    Meter.of(4)),
                                                                                                                                          Rotation2d.fromDegrees(0)));
        swerveDrive.resetDriveEncoders();
        swerveDrive.setModuleEncoderAutoSynchronize(true, 1);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }

      if (Robot.isSimulation()){
      swerveDrive.setHeadingCorrection(false);
      swerveDrive.setCosineCompensator(false);
      }
    }
    public void resetHeading(double remplaceHeading){
        swerveDrive.setGyro(new Rotation3d(Rotation2d.fromDegrees(remplaceHeading)));
        swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(remplaceHeading)));
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle Absolute 1", swerveDrive.getStates()[0].angle.getDegrees());
    SmartDashboard.putNumber("Angle Absolute 2", swerveDrive.getStates()[1].angle.getDegrees());
    SmartDashboard.putNumber("Angle Absolute 3", swerveDrive.getStates()[2].angle.getDegrees());
    SmartDashboard.putNumber("Angle Absolute 4", swerveDrive.getStates()[3].angle.getDegrees());

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
