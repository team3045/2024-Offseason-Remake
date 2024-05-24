// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private TalonFX leftClimber;
  private TalonFX rightClimber;

  /*Mechanism2d stuff for Sim and Dashboard */
  private MechanismLigament2d leftMechanismLigament;
  private MechanismLigament2d rightMechanismLigament;
  private MechanismRoot2d leftMechanismRoot;
  private MechanismRoot2d rightMechanismRoot;
  
  /*3d Pose Loggers */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable climberTable = inst.getTable("Climber");
  private final StructPublisher<Pose3d> sectionOnePublisher = climberTable.getStructTopic("Sec One Pose3d", Pose3d.struct).publish(); 
  private final StructPublisher<Pose3d> sectionTwoPublisher = climberTable.getStructTopic("Sec Two Pose3d", Pose3d.struct).publish(); 
  private final StructPublisher<Pose3d> sectionThreePublisher = climberTable.getStructTopic("Sec THree Pose3d", Pose3d.struct).publish(); 
  
  private Mechanism2d mechanism2d;

  /*Simulation stuff */
  private static ElevatorSim leftElevatorSim = new ElevatorSim(
      DCMotor.getFalcon500(1).withReduction(ClimberConstants.gearing), 
      1, 
      ClimberConstants.carriageMassKg, 
      ClimberConstants.drumRadiusMeters, 
      ClimberConstants.minHeightMeters, 
      ClimberConstants.maxHeightMeters, 
      true, 
      ClimberConstants.minHeightMeters);

  private static ElevatorSim rightElevatorSim = new ElevatorSim(
    DCMotor.getFalcon500(1).withReduction(ClimberConstants.gearing), 
      1, 
    ClimberConstants.carriageMassKg, 
    ClimberConstants.drumRadiusMeters, 
    ClimberConstants.minHeightMeters, 
    ClimberConstants.maxHeightMeters, 
    true, 
    ClimberConstants.minHeightMeters);;

  private TalonFXSimState leftSim;
  private TalonFXSimState rightSim;

  /*Climber state */
  private enum ClimberState {
    Down,
    Up
  }
  private ClimberState myState;

  /* Triggers for Coordinating Subsystems  */
  public final Trigger climbersUpTrigger = new Trigger(() -> myState == ClimberState.Up);
  public final Trigger climbersDownTrigger = new Trigger(() -> myState == ClimberState.Down);


  /** Creates a new Climber. */
  public Climber() {
    leftClimber = new TalonFX(ClimberConstants.leftClimberID, ClimberConstants.climberCanbus);
    rightClimber = new TalonFX(ClimberConstants.rightClimberID, ClimberConstants.climberCanbus);

    configMotors();
    setUpMechanism();

    myState = ClimberState.Down;

    if(Utils.isSimulation())
      setUpSim();
  }

  public void configMotors(){
    /*Motor Config */
    leftClimber.clearStickyFaults();
    rightClimber.clearStickyFaults();

    leftClimber.getConfigurator().apply(
      ClimberConstants.motorConfig.
      withMotorOutput(ClimberConstants.motorOutputConfigs.
        withInverted(ClimberConstants.leftInvert)));
    
    rightClimber.getConfigurator().apply(
      ClimberConstants.motorConfig.
      withMotorOutput(ClimberConstants.motorOutputConfigs.
        withInverted(ClimberConstants.rightInvert)));

    leftClimber.getPosition().setUpdateFrequency(50);
    rightClimber.getPosition().setUpdateFrequency(50);
  }

  public void setUpSim(){
    leftElevatorSim.setState(ClimberConstants.minHeightMeters, 0);
    rightElevatorSim.setState(ClimberConstants.minHeightMeters, 0);

    leftSim = leftClimber.getSimState();
    rightSim = rightClimber.getSimState();

    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    leftSim.Orientation = ChassisReference.Clockwise_Positive;
    rightSim.Orientation = ChassisReference.CounterClockwise_Positive;
  }

  public void setUpMechanism(){
    mechanism2d = new Mechanism2d(ClimberConstants.canvasWidth, ClimberConstants.canvasHeight);

    leftMechanismRoot = mechanism2d.getRoot("leftClimber", ClimberConstants.leftRootX, ClimberConstants.rootY);
    rightMechanismRoot = mechanism2d.getRoot("rightClimber", ClimberConstants.rightRootX, ClimberConstants.rootY);

    leftMechanismLigament = leftMechanismRoot.append(
      new MechanismLigament2d("leftLigament", getHeightMeters(), 90));
    rightMechanismLigament = rightMechanismRoot.append(
      new MechanismLigament2d("rightLigament", getHeightMeters(), 90));
  }

  public void updateMechanism(){
    leftMechanismLigament.setLength(getHeightMeters());
    rightMechanismLigament.setLength(getHeightMeters());

    SmartDashboard.putData("/Climber/Mechanism", mechanism2d);
    SmartDashboard.putNumber("/Climber/Height", getHeightMeters());
    SmartDashboard.putString("/Climber/State", myState.toString());
  }

  public void log3dPoses(){
    sectionOnePublisher.set(ClimberConstants.rightBasePoseSec1);
    sectionTwoPublisher.set(ClimberConstants.rightBasePoseSec2);
    sectionThreePublisher.set(ClimberConstants.rightBasePoseSec3);

    sectionTwoPublisher.set(ClimberConstants.rightBasePoseSec2.transformBy(new Transform3d(0,0,ClimberConstants.sectionTwoStart,new Rotation3d())));
    sectionThreePublisher.set(ClimberConstants.rightBasePoseSec3.transformBy(new Transform3d(0, 0,ClimberConstants.sectionThreeStart, new Rotation3d())));
    // if(getHeightMeters() <= ClimberConstants.sectionTwoStart){
    //   sectionTwoPublisher.set(ClimberConstants.rightBasePoseSec1);
    //   sectionThreePublisher.set(ClimberConstants.rightBasePoseSec1);
    // }
    // else if (getHeightMeters() > ClimberConstants.sectionTwoStart && getHeightMeters() < ClimberConstants.sectionThreeStart){
    //   sectionTwoPublisher.set(new Pose3d(
    //     new Translation3d(0, 0,getHeightMeters() - ClimberConstants.sectionTwoStart),
    //     new Rotation3d()));
    //   sectionThreePublisher.set(ClimberConstants.rightBasePoseSec1);
    // }
    // else{
    //   sectionTwoPublisher.set(new Pose3d(
    //     new Translation3d(0,0,ClimberConstants.sectionTwoStart),
    //     new Rotation3d()));
    //   sectionThreePublisher.set(new Pose3d(
    //     new Translation3d(0, 0,getHeightMeters() - ClimberConstants.sectionThreeStart),
    //     new Rotation3d()));
    // }
  }


  //Assumption that both Climbers always move together or atleast close, getPosition takes into account gearing
  public double getHeightMeters(){
    return leftClimber.getPosition().getValueAsDouble() * ClimberConstants.circumference;
  }

  /*State */
  public ClimberState getState(){
    return myState;
  }

  public void setState(ClimberState state){
    myState = state;
  }

  private void resetState(){
    if(getHeightMeters() > ClimberConstants.maxHeightMeters)
      myState = ClimberState.Up;
    else 
      myState = ClimberState.Down;
  }
  

  /*Base Control method for height in */
  public void goToHeight(double desiredHeightMeters){
    if(desiredHeightMeters > ClimberConstants.maxHeightMeters){
      System.out.println("Height too high");
      goToHeight(ClimberConstants.maxHeightMeters);
      return;
    }
    if(desiredHeightMeters < ClimberConstants.minHeightMeters){
      System.out.println("Height too low");
      goToHeight(ClimberConstants.maxHeightMeters);
      return;
    }

    double desiredRot = desiredHeightMeters / ClimberConstants.circumference;
    SmartDashboard.putNumber("/Climber/Desired Height", desiredHeightMeters);
    SmartDashboard.putNumber("/Climber/Desired Rot", desiredRot);
    MotionMagicVoltage request = new MotionMagicVoltage(desiredRot);
    leftClimber.setControl(request.withSlot(0));
    rightClimber.setControl(request.withSlot(0));
  }

  /*General Controls */
  public void climbersUp(){
    goToHeight(ClimberConstants.climbingHeight);
    setState(ClimberState.Up);
  }

  public void climbersDown(){
    goToHeight(ClimberConstants.minHeightMeters);
    setState(ClimberState.Down);
  }

  public void decreaseHeight(){
    goToHeight(getHeightMeters() - 0.05);
  }

  public void increaseHeight(){
    goToHeight(getHeightMeters() + 0.05);
  }


  /*Periodics */
  @Override
  public void simulationPeriodic(){
    leftSim = leftClimber.getSimState();
    rightSim = rightClimber.getSimState();

    leftSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    rightSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double leftSimVoltage = leftSim.getMotorVoltage();
    double rightSimVoltage = rightSim.getMotorVoltage();

    leftElevatorSim.setInputVoltage(leftSimVoltage);
    rightElevatorSim.setInputVoltage(rightSimVoltage);

    leftElevatorSim.update(0.02);
    rightElevatorSim.update(0.02);

    leftSim.setRawRotorPosition((leftElevatorSim.getPositionMeters() / ClimberConstants.circumference) * ClimberConstants.gearing);
    rightSim.setRawRotorPosition((rightElevatorSim.getPositionMeters() / ClimberConstants.circumference) * ClimberConstants.gearing);
    leftSim.setRotorVelocity(leftElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);
    rightSim.setRotorVelocity(rightElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);

    SmartDashboard.putNumber("/Climber/Left/RotorPos", leftClimber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("/Climber/Right/RotorPos", rightClimber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("/Climber/Left/RotorVelo", leftClimber.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("/Climber/Right/RotorVelo", rightClimber.getVelocity().getValueAsDouble());

    updateMechanism();
    log3dPoses();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMechanism();
    log3dPoses();
  }

}
