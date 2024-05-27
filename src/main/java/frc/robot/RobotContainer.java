// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.VisionSub;
import frc.robot.Vision.CameraBase;
import frc.robot.Vision.GremlinLimelightCam;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandPS4Controller joystick = new CommandPS4Controller(0); // My joystick
  private static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  /*Cameras */
  private static final CameraBase[] cameras = {
    new GremlinLimelightCam("front", VisionConstants.frontTransform, VisionConstants.frontCamTrust, VisionConstants.LL3_1280_720()),
    new GremlinLimelightCam("intake", VisionConstants.intakeTransform, VisionConstants.frontCamTrust, VisionConstants.LL3_1280_720())
  };

  /*Subsystems */
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  public static final VisionSub vision = new VisionSub(cameras, drivetrain);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.15).withRotationalDeadband(MaxAngularRate * 0.15) // Add a 15% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.square().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.cross().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.L2().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    /*Shooter up and down */
    joystick.R1().whileTrue(shooter.runOnce(() -> shooter.increaseAngle()).repeatedly());
    joystick.L1().whileTrue(shooter.runOnce(() -> shooter.decreaseAngle()).repeatedly());
    joystick.triangle().onTrue(shooter.goMaxAngle());
    joystick.circle().onTrue(shooter.goMinAngle());

    joystick.L2().toggleOnTrue(shooter.getShooterMaxSpeedCommand());
    joystick.L2().toggleOnFalse(shooter.stopCommand());

    joystick.R2().toggleOnTrue(
      Commands.parallel(
        shooter.goIntakeAngle(),
        Commands.waitUntil(shooter::atIntake)
          .andThen(intake::runIntakeMotor)
          .andThen(intake::runFeedMotor)
          .until(intake::noteDetected)
      ).andThen(intake::stopBoth)
    );

  }

  public RobotContainer() {
    configureBindings();

    /*Update Pose with vision measurements*/
    vision.periodic();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
