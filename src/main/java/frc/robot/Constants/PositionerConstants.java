package frc.robot.Constants;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class PositionerConstants {
    /*Motor IDs and Canbus */
    public static final int leftMotorId = 15;
    public static final int rightMotorId = 16;
    public static final int cancoderId = 17;
    public static final String canbus = "3045 Canivore";


    /*Offsets and Constants */
    public static final double canvasWidth = Units.inchesToMeters(26.5); //arbitray units but meters
    public static final double canvasHeight = Units.inchesToMeters(48); //arbitrary units but meter
    public static final double length = Units.inchesToMeters(18); //match the canvasWidth and canvasHeight units, meters
    public static final double cancoderOffset = 0.25; //rotations
    public static final SensorDirectionValue cancoderDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double sensorToMechanismRatio = 1;
    public static final double feedbackRotorOffset = 0;
    public static final InvertedValue rightInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue leftInvert = InvertedValue.CounterClockwise_Positive;
    public static final double gearing = (70 / 26) * (5) * (4) *(1/1.4);
    public static final double armRootX = Units.inchesToMeters(13.25 - 3.25);
    public static final double armRootY = Units.inchesToMeters(9);

    /*Simulation & Physics*/
    public static final DCMotor motor = DCMotor.getKrakenX60(2);
    public static final double mass = 1; //Kg
    public static final double momentOfInertia = 0.2525; //jKg meters squared
    public static final double pitchOffset = Units.degreesToRadians(50);
    public static final double xOrigin = Units.inchesToMeters(3.25);
    public static final double yOrigin = 0;
    public static final double zOrigin = Units.inchesToMeters(13);


    /*PID Values */
    public static final double pGain = 100;
    public static final double dGain = 0;
    public static final double iGain = 0;

    /*Constraints */
    public static final double maxAngle = 60;
    public static final double minAngle = 0;
    public static final double maxVelo = 3;
    public static final double maxAccel = 30; //reach in 0.1s 
   

    /*Configs */
    public static final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
      .withMagnetOffset(PositionerConstants.cancoderOffset)
      .withSensorDirection(PositionerConstants.cancoderDirection);

    public static final CANcoderConfiguration canCoderConfig = new CANcoderConfiguration().withMagnetSensor(magnetSensorConfigs);

    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); 
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicAcceleration(maxAccel).withMotionMagicCruiseVelocity(maxVelo);
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake); //Dont include invert here
    public static final VoltageConfigs voltageConfigs = new VoltageConfigs();

    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackRemoteSensorID(cancoderId).withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withSensorToMechanismRatio(sensorToMechanismRatio).withFeedbackRotorOffset(feedbackRotorOffset).withRotorToSensorRatio(gearing);
    
    public static final Slot0Configs slot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine).withKP(pGain).withKI(dGain).withKD(iGain);
      
    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
      .withAudio(new AudioConfigs())
      .withClosedLoopGeneral(new ClosedLoopGeneralConfigs())
      .withClosedLoopRamps(new ClosedLoopRampsConfigs())
      .withCurrentLimits(currentLimitsConfigs)
      .withCustomParams(new CustomParamsConfigs())
      .withDifferentialConstants(new DifferentialConstantsConfigs())
      .withDifferentialSensors(new DifferentialSensorsConfigs())
      .withFeedback(feedbackConfigs)
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs())
      .withMotionMagic(motionMagicConfigs)
      .withMotorOutput(motorOutputConfigs)
      .withOpenLoopRamps(new OpenLoopRampsConfigs())
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs())
      .withTorqueCurrent(new TorqueCurrentConfigs())
      .withVoltage(voltageConfigs)
      .withSlot0(slot0Configs);

}
