// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DistanceSensorReader;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorId, IntakeConstants.canbus);
  private TalonFX feedMotor = new TalonFX(IntakeConstants.feedMotorId, IntakeConstants.canbus);
  private DistanceSensorReader rangeSensorReader = new DistanceSensorReader();

  /*Run on seperate threat to avoid loop overruns */
  private Notifier rangeSensorNotifier = new Notifier(rangeSensorReader);

  /** Creates a new Intake. */
  public Intake() {
    rangeSensorReader.run();
    rangeSensorNotifier.setName("Range Sensors");
    rangeSensorNotifier.startPeriodic(0.02);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeMotor(){
    return this.run(() -> intakeMotor.set(IntakeConstants.intakeSpeed));
  }

  public Command runFeedMotor(){
    return this.run(() -> feedMotor.set(IntakeConstants.feedSpeed));
  }

  public Command stopIntakeMotor(){
    return this.run(() -> intakeMotor.stopMotor());
  }

  public Command stopFeedMotor(){
    return this.run(() -> feedMotor.stopMotor());
  }

  public Command stopBoth(){
    return stopIntakeMotor().alongWith(stopFeedMotor());
  }

  public boolean noteDetected(){
    return rangeSensorReader.getRange() > IntakeConstants.rangeSensorThreshold;
  }
}
