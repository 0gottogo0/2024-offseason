// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Aim;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 3 * Math.PI; // Tune

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController DriverController = new CommandXboxController(0); // My DriverController
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain



  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  
  private void configureBindings() {

    Aim aim = new Aim();

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
        drive.withVelocityX(
          MathUtil.applyDeadband(DriverController.getLeftY(), 0.15) * MaxSpeed) // Drive forward with negative Y (forward)
             .withVelocityY(MathUtil.applyDeadband(DriverController.getLeftX(), 0.15) * MaxSpeed) // Drive left with negative X (left)
             .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    DriverController.button(8).whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    DriverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    DriverController.b().whileTrue(aim.runOnce(() -> aim.setAimAtSpeaker()));

    DriverController.a().whileTrue(aim.runOnce(() -> aim.setAimUnderStage()));
  }
 
  /*
  Idk ill fix this

  aim.setDefaultCommand(aim.run(
    () -> aim.setAimJoystick((manipulatorController.getLeftY()))));
  */

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
