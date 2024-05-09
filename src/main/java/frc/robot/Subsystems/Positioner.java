// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PositionerConstants;

public class Positioner extends SubsystemBase {
  private Rotation2d mechanismAngle;

  private TalonFX leftSideMotor = new TalonFX(PositionerConstants.leftMotorId, PositionerConstants.canbus);
  private TalonFX rightSideMotor = new TalonFX(PositionerConstants.rightMotorId, PositionerConstants.canbus);
  private CANcoder cancoder = new CANcoder(PositionerConstants.cancoderId, PositionerConstants.canbus);

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
    true, //simulate gravity 
    0);

  /*Publishing */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable armTable = inst.getTable("Positioner");
  private final StructPublisher<Pose3d> pose3dPublisher = armTable.getStructTopic("Arm Pose3d", Pose3d.struct).publish();

  private TalonFXSimState leftSim;
  private TalonFXSimState rightSim;
  private CANcoderSimState cancoderSim;


  /** Creates a new Positioner. */
  public Positioner() {
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
  }

  public void configDevices(){
    /*Cancoder Config */
    cancoder.clearStickyFaults();
    cancoder.getConfigurator().apply(PositionerConstants.canCoderConfig);

    /*Motor Config */
    leftSideMotor.clearStickyFaults();
    rightSideMotor.clearStickyFaults();

    leftSideMotor.getConfigurator().apply(
      PositionerConstants.motorConfig.
      withMotorOutput(PositionerConstants.motorOutputConfigs.
        withInverted(PositionerConstants.leftInvert)));
    
    rightSideMotor.getConfigurator().apply(
      PositionerConstants.motorConfig.
      withMotorOutput(PositionerConstants.motorOutputConfigs.
        withInverted(PositionerConstants.rightInvert)));  

    rightSideMotor.getPosition().setUpdateFrequency(100);
    leftSideMotor.getPosition().setUpdateFrequency(100);
    cancoder.getPosition().setUpdateFrequency(100);
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

    SmartDashboard.putData("/Positioner/Arm/Mechanism", mechanism2d);
    SmartDashboard.putNumber("/Positioner/Arm/Mechanism", getArmAngleDegrees());
    double pitch = -Units.degreesToRadians(mechanismLigament.getAngle()) + PositionerConstants.pitchOffset;

    pose3dPublisher.set(new Pose3d(
      new Translation3d(PositionerConstants.xOrigin,PositionerConstants.yOrigin,PositionerConstants.zOrigin), 
      new Rotation3d(0,pitch,0)));
  }

  public double getArmAngleRotations(){
    return cancoder.getPosition().getValueAsDouble();
  }

  public double getArmAngleDegrees(){
    return Units.rotationsToDegrees(getArmAngleRotations());
  }

  public double getMotorAngleRotations(){
    return leftSideMotor.getPosition().getValueAsDouble();
  }

  //In degrees
  public void goToAngle(double desiredAng){
    if(desiredAng > PositionerConstants.maxAngle){
      System.out.println("Angle too High");
      goToAngle(PositionerConstants.maxAngle);
      return;
    }
    if(desiredAng < PositionerConstants.minAngle){
      System.out.println("Angle too Low");
      goToAngle(PositionerConstants.minAngle);
      return;
    }
    

    double desiredRot = Units.degreesToRotations(desiredAng);
    MotionMagicVoltage request = new MotionMagicVoltage(desiredRot);
    System.out.println("Going to Angle: " + desiredAng);
    
    
    leftSideMotor.setControl(request.withSlot(0));
    rightSideMotor.setControl(request.withSlot(0));

  }

  public void increaseAngle(){
    goToAngle(getArmAngleDegrees()+2);
  }

  public void decreaseAngle(){
    goToAngle(getArmAngleDegrees()-2);
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
}
