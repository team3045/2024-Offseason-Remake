// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DistanceSensorReader;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId, IntakeConstants.canbus);
  private TalonFX feedMotor = new TalonFX(IntakeConstants.feedMotorId, ShooterConstants.canbus);
  private DistanceSensorReader rangeSensorReader = new DistanceSensorReader();

  /*Run on seperate threat to avoid loop overruns */
  private Notifier rangeSensorNotifier = new Notifier(rangeSensorReader);

  /** Creates a new Intake. */
  public Intake() {
    rangeSensorReader.run();
    rangeSensorNotifier.setName("Range Sensors");
    rangeSensorNotifier.startPeriodic(0.02);

    feedMotor.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    intakeMotor.getConfigurator().apply(
      new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeMotor(){
    System.out.println("RUNNING INTAKE");
    return this.run(() -> intakeMotor.set(IntakeConstants.intakeSpeed));
  }

  public Command runFeedMotor(){
    return this.run(() -> feedMotor.set(IntakeConstants.feedSpeed));
  }

  public Command runBoth(){
    return this.run(() -> {
      feedMotor.set(IntakeConstants.feedSpeed);
      intakeMotor.set(IntakeConstants.intakeSpeed);
    });
  }

  public Command stopIntakeMotor(){
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command stopFeedMotor(){
    return this.run(() -> feedMotor.stopMotor());
  }

  public Command stopBoth(){
    return this.run(() -> {
      feedMotor.stopMotor();
      intakeMotor.stopMotor();
    });
  }

  public void stop(){
    feedMotor.stopMotor();
    intakeMotor.stopMotor();
  }

  public void stopFeedRunnable(){
    feedMotor.stopMotor();
  }

  public Command runBack(){
    System.out.println("RUN BACKKKKKK");

    return this.run(() -> feedMotor.set(-0.1));
  }

  public boolean noteDetected(){
    return rangeSensorReader.getRange() < IntakeConstants.rangeSensorThreshold;
  }
}
