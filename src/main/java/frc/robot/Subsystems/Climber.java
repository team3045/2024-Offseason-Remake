// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PositionerConstants;

public class Climber extends SubsystemBase {
  private TalonFX leftClimber;
  private TalonFX rightClimber;

  /*Mechanism2d stuff for Sim and Dashboard */
  private MechanismLigament2d leftMechanismLigament;
  private MechanismLigament2d rightMechanismLigament;
  private MechanismRoot2d leftMechanismRoot;
  private MechanismRoot2d rightMechanismRoot;
  
  private Mechanism2d mechanism2d;

  /*Simulation stuff */
  private static ElevatorSim leftElevatorSim = new ElevatorSim(
      DCMotor.getFalcon500(1), 
      ClimberConstants.gearing, 
      ClimberConstants.carriageMassKg, 
      ClimberConstants.drumRadiusMeters, 
      ClimberConstants.minHeightMeters, 
      ClimberConstants.maxHeightMeters, 
      true, 
      ClimberConstants.minHeightMeters);

  private static ElevatorSim rightElevatorSim = new ElevatorSim(
    DCMotor.getFalcon500(1), 
    ClimberConstants.gearing, 
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

    leftSim.Orientation = ChassisReference.CounterClockwise_Positive;
    rightSim.Orientation = ChassisReference.Clockwise_Positive;
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

    if(Utils.isSimulation()){
      leftMechanismLigament.setLength(leftElevatorSim.getPositionMeters());
      rightMechanismLigament.setLength(rightElevatorSim.getPositionMeters());
    }

    SmartDashboard.putData("/Climber/Mechanism", mechanism2d);
    SmartDashboard.putNumber("/Climber/Height", getHeightMeters());
    SmartDashboard.putString("/Climber/State", myState.toString());
  }


  //Assumption that both Climbers always move together or atleast close, getPosition takes into account gearing
  public double getHeightMeters(){
    return leftClimber.getPosition().getValueAsDouble() * ClimberConstants.circumference + ClimberConstants.minHeightMeters;
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
      goToHeight(ClimberConstants.maxHeightMeters);
      return;
    }
    if(desiredHeightMeters < ClimberConstants.minHeightMeters){
      System.out.println("Angle too Low");
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
    goToHeight(getHeightMeters() - 0.2);
  }

  public void increaseHeight(){
    goToHeight(getHeightMeters() + 0.2);
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

    leftSim.setRawRotorPosition(leftElevatorSim.getPositionMeters() / ClimberConstants.circumference);
    rightSim.setRawRotorPosition(rightElevatorSim.getPositionMeters() / ClimberConstants.circumference);
    leftSim.setRotorVelocity(leftElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);
    rightSim.setRotorVelocity(rightElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);

    SmartDashboard.putNumber("/Climber/Left/RotorPos", leftElevatorSim.getPositionMeters() / ClimberConstants.circumference);
    SmartDashboard.putNumber("/Climber/Right/RotorPos", rightElevatorSim.getPositionMeters() / ClimberConstants.circumference);
    SmartDashboard.putNumber("/Climber/Left/RotorVelo", leftElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);
    SmartDashboard.putNumber("/Climber/Right/RotorVelo", rightElevatorSim.getVelocityMetersPerSecond() / ClimberConstants.circumference);

    System.out.println("Sim Pos: " + leftElevatorSim.getPositionMeters());
    System.out.println("Sim velo: " + leftElevatorSim.getVelocityMetersPerSecond());
    System.out.println("Sim output: " + leftElevatorSim.getOutput(0));
    

    updateMechanism();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateMechanism();
  }

}
