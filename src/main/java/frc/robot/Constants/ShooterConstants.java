// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.FRCNetComm.tResourceType;

/** Add your docs here. */
public class ShooterConstants {
    public static final int topMotorId = 14;
    public static final int botMotorId = 15;
    public static final String canbus = "rio";

    public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutral = NeutralModeValue.Coast;

    public static final double gearing = (12.0) / (18.0);

    /*Current Limits */
    public static final double statorCurrentLimit = 60; //Amps
    public static final double supplyCurrentLimit = 40;
    public static final double supplyCurrentThreshold = 60;
    public static final double supplyCurrentTime = 0.1; //Seconds

    /*Slot 0*/ 
    public static final double kS = 0.1146;
    public static final double kA = 0.0036132;
    public static final double kV = 0.082478;
    public static final double kP = 1;

    /*Max Speed */
    public static double topMax = 145 * (0.95);
    public static double botMax = 145 * (0.9);


    /*Motion Magic */
    public static final double maxAccel = 400; //Rot per Sec^2

    public static final TalonFXConfiguration config = 
        new TalonFXConfiguration().
            withCurrentLimits(new CurrentLimitsConfigs().
                withStatorCurrentLimit(statorCurrentLimit).
                withSupplyCurrentLimit(supplyCurrentLimit).
                withSupplyCurrentThreshold(supplyCurrentThreshold).
                withSupplyTimeThreshold(supplyCurrentTime).
                withStatorCurrentLimitEnable(true).withSupplyCurrentLimitEnable(true)).
            withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(maxAccel)).
            withMotorOutput(new MotorOutputConfigs()
                .withInverted(inverted)
                .withNeutralMode(neutral)).
            withSlot0(new Slot0Configs()
                .withKA(kA)
                .withKV(kV)
                .withKP(kP)
                .withKS(kS)).
            withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearing));
}
