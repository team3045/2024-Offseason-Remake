// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ClimberConstants {
    /*Can IDs and Canbus*/
    public static final int leftClimberID = 18;
    public static final int rightClimberID = 19;
    public static final String climberCanbus = "";

    /*Gearbox & Inverts*/
    public static final double gearing = 30;
    public static final InvertedValue leftInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightInvert = InvertedValue.CounterClockwise_Positive;

    /*Mechanism2d */
    public static final double canvasWidth = 4;//arbitray units but meters
    public static final double canvasHeight = 4; //arbitrary units but meter
    public static final double leftRootX = 2;
    public static final double rightRootX = 3;
    public static final double rootY = 0;

    /*PID Values */
    public static final double pGain = 20;
    public static final double dGain = 0;
    public static final double iGain = 0;

    /*Constraints */
    public static final double maxVelo = 1;
    public static final double maxAccel = 10; //0.1s to get to max speed


    /*Physics */
    public static final double carriageMassKg = 1; 
    public static final double drumRadiusMeters = 0.05 / Math.PI;
    public static final double circumference = 2 * drumRadiusMeters * Math.PI;

    /*Sections */
    public static final double sectionOneLength = Units.inchesToMeters(12);
    public static final double sectionTwoLength = Units.inchesToMeters(7);
    public static final double sectionThreeLength = Units.inchesToMeters(9);
    public static final double sectionOneStart = 0;
    public static final double sectionTwoStart = sectionOneStart + sectionOneLength;
    public static final double sectionThreeStart = sectionTwoStart + sectionTwoLength;
    public static final Pose3d rightBasePoseSec1 = new Pose3d(new Translation3d(-0.12,0.22,0.1), new Rotation3d(0,0,Units.degreesToRadians(180)));
    public static final Pose3d rightBasePoseSec2 = new Pose3d(new Translation3d(-0.10,0.28,0.1), new Rotation3d(0,0,Units.degreesToRadians(180)));
    public static final Pose3d rightBasePoseSec3 = new Pose3d(new Translation3d(-0.24,0.20,0.1), new Rotation3d(0,0,Units.degreesToRadians(0)));
    public static final Pose3d leftBasePose = new Pose3d(new Translation3d(-0.25,-0.2,0.1), new Rotation3d());


    /*Heights */
    public static final double minHeightMeters = 1;
    public static final double maxHeightMeters = 3; 
    public static final double climbingHeight = 2;


    /*Motor Configs */
    public static final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs(); 
    public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
      .withMotionMagicAcceleration(maxAccel).withMotionMagicCruiseVelocity(maxVelo);
    public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake); //Dont include inverted here
    public static final VoltageConfigs voltageConfigs = new VoltageConfigs();
    public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(gearing);
    public static final Slot0Configs slot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static).withKP(pGain).withKI(dGain).withKD(iGain);
    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
      
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
      .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
      .withTorqueCurrent(new TorqueCurrentConfigs())
      .withVoltage(voltageConfigs)
      .withSlot0(slot0Configs);
}
