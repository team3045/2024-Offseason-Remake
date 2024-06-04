// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PositionerConstants;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;

import java.security.CodeSigner;

public class Shooter extends SubsystemBase {
  private Rotation2d mechanismAngle;
  private double desiredAngle;

  private TalonFX leftSideMotor = new TalonFX(PositionerConstants.leftMotorId, PositionerConstants.canbus);
  private TalonFX rightSideMotor = new TalonFX(PositionerConstants.rightMotorId, PositionerConstants.canbus);
  private CANcoder cancoder = new CANcoder(PositionerConstants.cancoderId, PositionerConstants.canbus);
  private TalonFX topShooterMotor = new TalonFX(ShooterConstants.topMotorId, ShooterConstants.canbus);
  private TalonFX botShooterMotor = new TalonFX(ShooterConstants.botMotorId, ShooterConstants.canbus);

  /*Mechanism2d stuff for Sim and Dashboard */
  private MechanismLigament2d mechanismLigament;
  private MechanismRoot2d mechanismRoot;
  private Mechanism2d mechanism2d;

  /*Simulation */
  private static SingleJointedArmSim armSim = new SingleJointedArmSim(
    PositionerConstants.motor, 
    PositionerConstants.gearing, 
    PositionerConstants.momentOfInertia, 
    PositionerConstants.length, 
    Units.degreesToRadians(PositionerConstants.minAngle), 
    Units.degreesToRadians(PositionerConstants.maxAngle), 
    true,  
    0);

  /*Publishing */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable armTable = inst.getTable("Positioner");
  private final StructPublisher<Pose3d> pose3dPublisher = armTable.getStructTopic("Arm Pose3d", Pose3d.struct).publish();

  private TalonFXSimState leftSim;
  private TalonFXSimState rightSim;
  private CANcoderSimState cancoderSim;

  /*Sysid Values */
  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Angle> appliedAngle = MutableMeasure.mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> appliedVelocity = MutableMeasure.mutable(RotationsPerSecond.of(0));
  private final MutableMeasure<Current> appliedCurrent = MutableMeasure.mutable(Amps.of(0));
  private final Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
  private final Measure<Velocity<Current>> currentRampRate = Amps.of(0.2).per(Seconds.of(1));
  private final SysIdRoutine.Config sysidConfig = new SysIdRoutine.Config(
    rampRate, 
    Volts.of(10), //30 Amps 
    Seconds.of(30),
    null);
  private SysIdRoutine armRoutine;
  private SysIdRoutine flywheelRoutine;
  private double current;
 
  /** Creates a new Positioner. */
  public Shooter() {
    configDevices();

    /*Set up mechanism */
    mechanismAngle = Rotation2d.fromRotations(cancoder.getPosition().getValue());

    mechanism2d = new Mechanism2d(PositionerConstants.canvasWidth, PositionerConstants.canvasHeight);
    mechanismRoot = mechanism2d.getRoot("positioner", PositionerConstants.armRootX, PositionerConstants.armRootY);
    mechanismLigament = mechanismRoot.append(
      new MechanismLigament2d("ligament", PositionerConstants.length, mechanismAngle.getDegrees())); //Starts at angle 0

    /*Set up Simulation */
    if(Utils.isSimulation()){
      setUpSim();
    }

    /*Set Up Sysid */
    armRoutine = new SysIdRoutine(
      sysidConfig , 
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> setCurrent(volts.in(Volts)),
        log -> {
          log.motor("LeftSideMotor")
            .voltage(appliedVoltage.mut_replace(leftSideMotor.getTorqueCurrent().getValueAsDouble(), Volts))
            .angularPosition(appliedAngle.mut_replace(leftSideMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(appliedVelocity.mut_replace(leftSideMotor.getVelocity().getValueAsDouble(), RotationsPerSecond))
            .current(appliedCurrent.mut_replace(leftSideMotor.getTorqueCurrent().getValueAsDouble(), Amps));
          log.motor("RightSideMotor")
            .voltage(appliedVoltage.mut_replace(rightSideMotor.getMotorVoltage().getValueAsDouble(), Volts))
            .angularPosition(appliedAngle.mut_replace(rightSideMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(appliedVelocity.mut_replace(rightSideMotor.getVelocity().getValueAsDouble(), RotationsPerSecond))
            .current(appliedCurrent.mut_replace(rightSideMotor.getTorqueCurrent().getValueAsDouble(), Amps));
        }, 
        this));
    flywheelRoutine = new SysIdRoutine(
      sysidConfig, 
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> volts) -> applyVoltageFlywheel(volts.in(Volts)), 
        log -> {
          log.motor("TopFlyWheelMotor")
            .voltage(appliedVoltage.mut_replace(topShooterMotor.getMotorVoltage().getValueAsDouble(), Volts))
            .angularPosition(appliedAngle.mut_replace(topShooterMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(appliedVelocity.mut_replace(topShooterMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
          log.motor("BotFlyWheelMotor")
            .voltage(appliedVoltage.mut_replace(botShooterMotor.getMotorVoltage().getValueAsDouble(), Volts))
            .angularPosition(appliedAngle.mut_replace(botShooterMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(appliedVelocity.mut_replace(botShooterMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
        }, 
        this)
    );

    current = 2.5;
  }

  public void configDevices(){
    /*Cancoder Config */
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(PositionerConstants.canCoderConfig);

    /*Motor Config */
    leftSideMotor.clearStickyFaults();
    rightSideMotor.clearStickyFaults();
    topShooterMotor.clearStickyFaults();
    botShooterMotor.clearStickyFaults();

    leftSideMotor.getConfigurator().apply(
      PositionerConstants.motorConfig.
      withMotorOutput(PositionerConstants.motorOutputConfigs.
        withInverted(PositionerConstants.leftInvert)));
    
    rightSideMotor.getConfigurator().apply(
      PositionerConstants.motorConfig.
      withMotorOutput(PositionerConstants.motorOutputConfigs.
        withInverted(PositionerConstants.rightInvert)));  

    rightSideMotor.getPosition().setUpdateFrequency(50);
    leftSideMotor.getPosition().setUpdateFrequency(50);
    cancoder.getPosition().setUpdateFrequency(50);

    topShooterMotor.getConfigurator().apply(ShooterConstants.config);
    botShooterMotor.getConfigurator().apply(ShooterConstants.config);
  }

  public void setUpSim(){
    armSim.setState(0, 0);

    leftSim = leftSideMotor.getSimState();
    rightSim = rightSideMotor.getSimState();
    cancoderSim = cancoder.getSimState();

    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightSim.Orientation = ChassisReference.Clockwise_Positive;
  }

  public void updateMechanism(){
    mechanismAngle = Rotation2d.fromRotations(getArmAngleRotations());
    mechanismLigament.setAngle(mechanismAngle);

    SmartDashboard.putData("/Positioner/Mechanism", mechanism2d);
    SmartDashboard.putNumber("/Positioner/Angle", getArmAngleDegrees());
    double pitch = -Units.degreesToRadians(mechanismLigament.getAngle()) + PositionerConstants.pitchOffset;

    pose3dPublisher.set(new Pose3d(
      new Translation3d(PositionerConstants.xOrigin,PositionerConstants.yOrigin,PositionerConstants.zOrigin), 
      new Rotation3d(0,pitch,0)));
  }

  public double getArmAngleRotations(){
    return cancoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getArmAngleDegrees(){
    return Units.rotationsToDegrees(getArmAngleRotations());
  }

  public double getMotorAngleRotations(){
    return leftSideMotor.getPosition().getValueAsDouble();
  }

  //In degrees
  public void requestAngle(double desiredAng){
    if(desiredAng > PositionerConstants.maxAngle){
      System.out.println("Angle too High");
      requestAngle(PositionerConstants.maxAngle);
      return;
    }
    if(desiredAng < PositionerConstants.minAngle){
      System.out.println("Angle too Low");
      requestAngle(PositionerConstants.minAngle);
      return;
    }

    desiredAngle = desiredAng; //update instance variable
    

    double desiredRot = Units.degreesToRotations(desiredAng);
 
    MotionMagicTorqueCurrentFOC MMRequest = new MotionMagicTorqueCurrentFOC(desiredRot);
    PositionTorqueCurrentFOC regRequest = new PositionTorqueCurrentFOC(desiredRot);
    
    SmartDashboard.putNumber("/Positioner/Desired Angle", desiredAng);
    
    if(Math.abs(desiredAng - getArmAngleDegrees()) < PositionerConstants.normalPIDThreshold){
      leftSideMotor.setControl(regRequest.withSlot(0).withUpdateFreqHz(1000));
      rightSideMotor.setControl(regRequest.withSlot(0).withUpdateFreqHz(1000));
    }
    else{
      leftSideMotor.setControl(MMRequest.withSlot(0).withUpdateFreqHz(1000));
      rightSideMotor.setControl(MMRequest.withSlot(0).withUpdateFreqHz(1000));
    }
  }

  public void increaseAngle(){
    requestAngle(getArmAngleDegrees()+2);
  }

  public void decreaseAngle(){
    requestAngle(getArmAngleDegrees()-2);
  }

  public void requestHold(){
    leftSideMotor.setControl(new StaticBrake());
    rightSideMotor.setControl(new StaticBrake());
  }

  @Override
  public void periodic() {
    updateMechanism();
  }
  
 @Override
  public void simulationPeriodic(){
    //Called once per scheduler run in simulation mode

    leftSim = leftSideMotor.getSimState();
    rightSim = rightSideMotor.getSimState();
    cancoderSim = cancoder.getSimState(); 
    
    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    cancoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double leftSimVoltage = leftSim.getMotorVoltage();


    armSim.setInputVoltage(leftSimVoltage);
    armSim.update(0.020);

    cancoderSim.setRawPosition(Units.radiansToRotations(armSim.getAngleRads()));
    leftSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads() / PositionerConstants.gearing));
    rightSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads() / PositionerConstants.gearing));

    leftSim.setRotorVelocity(Units.rotationsToRadians(armSim.getVelocityRadPerSec() / PositionerConstants.gearing));
    rightSim.setRotorVelocity(Units.rotationsToRadians(armSim.getVelocityRadPerSec() / PositionerConstants.gearing));

    updateMechanism();
  }

  /*FlyWheel Low Level Actions*/

  public void setShooterRPS(double topDesiredRPS, double botDesiredRPS){
    //VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(desiredRPS);

    VelocityVoltage request = new VelocityVoltage(0).withAcceleration(ShooterConstants.maxAccel);
    topShooterMotor.setControl(request.withVelocity(topDesiredRPS));
    botShooterMotor.setControl(request.withVelocity(botDesiredRPS));
  }

  public void setMaxSpeed(){
    topShooterMotor.set(0.95);
    botShooterMotor.set(0.9);
  }

  public void coastShooter(){
    topShooterMotor.setControl(new CoastOut());
    botShooterMotor.setControl(new CoastOut());
  }

  public boolean atAngle(double angle){
    return Math.abs(angle-getArmAngleDegrees()) < 1;
  }

  public boolean atIntake(){
    return atAngle(IntakeConstants.intakeAngle);
  }

  /*SysId Methods */
  public void applyVoltagePositioner(double volts){
    leftSideMotor.setVoltage(volts);
    rightSideMotor.setVoltage(volts);
  }

  public void applyVoltageFlywheel(double volts){
    topShooterMotor.setVoltage(volts);
    botShooterMotor.setVoltage(volts);
  }

  public Command quasiPositionerRoutineForward(){
    return Commands.print("QUASI Forward").andThen(armRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> getArmAngleDegrees() > PositionerConstants.maxAngle - 5));
  }

  public Command quasiPositionerRoutineReverse(){
    return Commands.print("QUASI Reverse")
      .andThen(armRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> getArmAngleDegrees() < PositionerConstants.minAngle + 5)
      .andThen(Commands.print("Finished")));
  }

  public Command dynaPositionerRoutineForward(){
    return armRoutine.dynamic(SysIdRoutine.Direction.kForward).until(() -> getArmAngleDegrees() > PositionerConstants.maxAngle - 5);
  }

  public Command dynaPositionerRoutineReverse(){
    return armRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> getArmAngleDegrees() < PositionerConstants.minAngle + 5);
  }

  public Command quasiFlywheelRoutine(SysIdRoutine.Direction direction){
    return flywheelRoutine.quasistatic(direction);
  }

  public Command dynaFlywheelRoutine(SysIdRoutine.Direction direction){
    return flywheelRoutine.dynamic(direction);
  }


  /*Command Factories */
  public Command getShooterSpeedCommand(double topDesiredRPS, double botDesiredRPS){
    return Commands.run(() -> setShooterRPS(topDesiredRPS, botDesiredRPS), this);
  }

  public Command getShooterMaxSpeedCommand(){
    return Commands.run(() -> setMaxSpeed(), this);
  }

  public Command stopCommand(){
    return Commands.run(() -> setShooterRPS(0, 0), this);
  }

  public Command goMaxAngle(){
    return Commands.run(() -> requestAngle(PositionerConstants.maxAngle), this);
  }

  public Command goMinAngle(){
    return Commands.run(() -> requestAngle(PositionerConstants.minAngle), this);
  }

  public Command goIntakeAngle(){
    return Commands.run(() -> requestAngle(IntakeConstants.intakeAngle), this);
  }

  public void increaseCurrent(){
    current+= 0.02;
    SmartDashboard.putNumber("Current Amps", current);
    
    leftSideMotor.setControl(new TorqueCurrentFOC(current));
    rightSideMotor.setControl(new TorqueCurrentFOC(current));
  }

  public void setCurrent(double current){
    leftSideMotor.setControl(new TorqueCurrentFOC(current));
    rightSideMotor.setControl(new TorqueCurrentFOC(current));
  }

  public void resetCurrent(){
    current = 2.5;
  }
}


