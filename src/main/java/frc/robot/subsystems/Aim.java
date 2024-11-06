// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Aim extends SubsystemBase {

  private TalonFX aim = new TalonFX(Constants.AimID);
  private DutyCycleEncoder aimEncoder = new DutyCycleEncoder(Constants.revEncoderDIOPort); 
  private PIDController aimController = new PIDController(0.175, 0F, 0F);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private double aimSetpoint = 110;

  /** Creates a new Aim. */
  public Aim() {

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    aim.clearStickyFaults();
    aim.getConfigurator().apply(cfg);

    aimController.setTolerance(.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.println(getAimAngleDeg());

    double pid = aimController.calculate(getAimAngleDeg(), aimSetpoint);
    pid = MathUtil.applyDeadband(pid, 0.05);
    aim.set(pid);
  }

  public double getAimAngleDeg() {
    return Units.rotationsToDegrees(-aimEncoder.getAbsolutePosition())+380;
  }

  // Fine tune setpoints
  public void setAimUnderStage() {
    aimSetpoint = 90;
  }

  public void setAimAtSpeaker() {
    aimSetpoint = 110;
  }
}
