// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class IntakeConstants {
    public static final int intakeMotorId = 13;
    public static final int feedMotorId = 2;
    public static final String canbus = "3045 Canivore";

    public static final double intakeSpeed = 0.4;
    public static final double feedSpeed = 0.4;

    public static final double intakeAngle = 45;

    public static final double rangeSensorThreshold = Units.inchesToMeters(4) * 1000; //mm
}
