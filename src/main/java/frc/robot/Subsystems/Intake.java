// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Notifier;
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
}
