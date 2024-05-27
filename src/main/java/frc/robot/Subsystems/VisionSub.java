// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision.CameraBase;
import frc.robot.Vision.GremlinLimelightCam;
import frc.robot.Vision.GremlinPhotonCam;
import frc.robot.Vision.LimelightHelpers;

public class VisionSub extends SubsystemBase {
  private CameraBase[] cameras;
  private CommandSwerveDrivetrain swerve;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot pose for field positioning */
  private final NetworkTable table = inst.getTable("Vision Poses");
  private final StructArrayPublisher<Pose2d> pose2dPublisher = table.getStructArrayTopic("Vision Poses 2d", Pose2d.struct).publish();
  private final StructArrayPublisher<Pose3d> pose3dPublisher = table.getStructArrayTopic("Vision Poses 3d", Pose3d.struct).publish();

  /*Simulation using Photonvision */
  private static VisionSystemSim visionSim;
  private PhotonCameraSim[] simCameras;

  /** Creates a new VisionSub. */
  public VisionSub(CameraBase[] cameras, CommandSwerveDrivetrain swerve) {
    this.cameras = cameras;
    this.swerve = swerve;
  }

  @Override
  public void periodic() {
    ArrayList<Pose2d> uPose2ds = new ArrayList<>();
    ArrayList<Pose3d> uPose3ds = new ArrayList<>();

    for(CameraBase cam : cameras){
      /*Update Photon Vision reference Pose*/
      if(cam instanceof GremlinPhotonCam){
        ((GremlinPhotonCam)cam).updateCurrPose(swerve.getState().Pose);
      }

      if(cam.getEstimatedPose3d().isPresent()){
        swerve.addVisionMeasurement(
          cam.getEstimatedPose3d().get().toPose2d(), 
          cam.getTimeStamp(),
          cam.getVisionStdDevs().copy()); //Copy will convert from vector to matrix TODO: find better solution later

        uPose2ds.add(cam.getEstimatedPose3d().get().toPose2d());
        uPose3ds.add(cam.getEstimatedPose3d().get());
      }
      else{
        uPose2ds.add(new Pose2d());
        uPose3ds.add(new Pose3d());
      }
    }

    pose2dPublisher.set(uPose2ds.toArray(new Pose2d[uPose2ds.size()]));
    pose3dPublisher.set(uPose3ds.toArray(new Pose3d[uPose3ds.size()]));
    
  }

  /*This will be called instead of periodic, runs on 10ms loop instead of 20 */
  public void updateVision(){
    ArrayList<Pose2d> uPose2ds = new ArrayList<>();
    ArrayList<Pose3d> uPose3ds = new ArrayList<>();

    for(CameraBase cam : cameras){
      if(cam.getEstimatedPose3d().isPresent()){
        swerve.addVisionMeasurement(
          cam.getEstimatedPose3d().get().toPose2d(), 
          cam.getTimeStamp(),
          cam.getVisionStdDevs().copy()); //Copy will convert from vector to matrix TODO: find better solution later

        uPose2ds.add(cam.getEstimatedPose3d().get().toPose2d());
        uPose3ds.add(cam.getEstimatedPose3d().get());
      }
      else{
        uPose2ds.add(new Pose2d());
        uPose3ds.add(new Pose3d());
      }
    }

    pose2dPublisher.set((Pose2d[]) uPose2ds.toArray());
    pose3dPublisher.set((Pose3d[]) uPose3ds.toArray());
  }

  public void initVisionSim() throws IOException{
    simCameras = new PhotonCameraSim[cameras.length];
    visionSim = new VisionSystemSim("Vision Sim");

    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide); //Our origin is always blue even if we're on red alliacne

    visionSim.addAprilTags(tagLayout);

    for(int i =0; i < cameras.length; i++){
      GremlinPhotonCam tempCam;
      if(cameras[i] instanceof GremlinLimelightCam){
        tempCam = new GremlinPhotonCam(
          cameras[i].getName(), 
          cameras[i].getRobotToCam(), 
          cameras[i].getTrustLevel(), 
          cameras[i].getCamProperties());
      }
      else{
        tempCam = (GremlinPhotonCam) cameras[i];
      }

      simCameras[i] = new PhotonCameraSim(tempCam.getPhotonCamera(), tempCam.getCamProperties());
      visionSim.addCamera(simCameras[i], tempCam.getRobotToCam());
    }

     
  }

  @Override
  public void simulationPeriodic(){
    visionSim.update(swerve.getState().Pose);

    var debugField = visionSim.getDebugField();
  }
}
