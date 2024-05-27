// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.Optional;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.Vision.LimelightHelpers.RawFiducial;

/** Add your docs here. */
public class GremlinLimelightCam implements CameraBase{
    private String name;
    private Transform3d robotToCam;
    private Vector<N3> lastStdDevs;
    private double lastTimestamp;
    private SimCameraProperties properties;
    private boolean seesSpeaker;

    /*Between 0 and 1 Lower Values are more trusted 1 is the normal Level*/
    private double trustLevel; 

    public GremlinLimelightCam(String name, Transform3d robotToCam, double trustLevel, SimCameraProperties properties){
        this.name = "limelight-" + name;
        this.robotToCam = robotToCam;
        this.trustLevel = trustLevel;
        this.properties = properties;

        lastStdDevs = CameraBase.baseVisionStdDev;
        lastTimestamp = Timer.getFPGATimestamp();
        seesSpeaker = false;

        if(name.equals("front"))
            LimelightHelpers.setPriorityTagID(
                "front", 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 7 : 3);
    }

    @Override
    public Vector<N3> getVisionStdDevs() {
        return lastStdDevs;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Optional<Pose3d> getEstimatedPose3d() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        calcStdDevs(estimate);
        updateTimestamp(estimate);

        if(lastStdDevs.elementSum() > CameraBase.throwoutStdDev.elementSum() - 1){
            return Optional.empty();
        }

        return Optional.of(new Pose3d(estimate.pose));
    }

    @Override
    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    /*Rather than directly throwout a PoseEstimate, we add everything to the kalman filter,
     *but if it doesnt pass filtering we add it with stdDevs high enough 
     *that will make the kalman filter essenbtially throw it out
     *Update: In Swerve sub where we add it, we will throwout rather than add anything with really high stdDevs
     */
    public void calcStdDevs(PoseEstimate estimate){
        /*throwout anything over a certain distance */
        if(estimate.avgTagDist > VisionConstants.throwoutDistanceMeters){
            lastStdDevs = CameraBase.throwoutStdDev;
            return;
        }

        //Make sure its within the field
        if(
            estimate.pose.getTranslation().getX() < VisionConstants.minFieldX ||
            estimate.pose.getTranslation().getX() > VisionConstants.maxFieldX ||
            estimate.pose.getTranslation().getY() < VisionConstants.minFieldY ||
            estimate.pose.getTranslation().getY() > VisionConstants.maxFieldY
        ) {
            lastStdDevs = CameraBase.throwoutStdDev;
            return;
        }

        RawFiducial[] fiducials = estimate.rawFiducials;

        lastStdDevs = VecBuilder.fill(VisionConstants.baseStdX, VisionConstants.baseStdY, VisionConstants.baseStdDeg);
        double xyStd = VisionConstants.baseStdX;

        xyStd *= trustLevel; //We might trust one camera more than another
        
        /*Scale based on distance, if over a certain threshold, less than that threshold we essentially assume is perfect*/
        if(estimate.avgTagDist > VisionConstants.trustedMaxDistance){
            double addedStd = VisionConstants.distanceModifier * estimate.avgTagDist;
            xyStd += addedStd;
        }


        /*Trust measurements with multiple tags more */
        if(estimate.tagCount > 1){
            xyStd /= VisionConstants.multTagDivisor;
        }

        /*Calculate ambiguity and check if any tags are trusted */
        double totalAmbiguity = 0;
        boolean trustedTag = false;
        for(RawFiducial tag : fiducials){
            if(VisionConstants.trustedTags.contains(tag.id) && !trustedTag){
                xyStd *= VisionConstants.trustedTagModifier;
                trustedTag = true;
            }
            totalAmbiguity += tag.ambiguity;
        }

        /*Throws out if over ambiguity threshold*/
        if(totalAmbiguity / estimate.tagCount > VisionConstants.throwoutAmbiguity){
            lastStdDevs = CameraBase.throwoutStdDev;
            return;
        }
        
        /*Increases stdDev if more ambiguous*/
        if(totalAmbiguity / estimate.tagCount > VisionConstants.trustedMaxAmbiguity){
            xyStd *= (1+totalAmbiguity / estimate.tagCount);
        }

        lastStdDevs = VecBuilder.fill(xyStd, xyStd, Units.degreesToRadians(90000));
    }  
    
    public void updateTimestamp(PoseEstimate estimate){
        lastTimestamp = estimate.timestampSeconds;
    }

    @Override
    public double getTimeStamp() {
        return lastTimestamp;
    }

    @Override
    public SimCameraProperties getCamProperties() {
        return properties;
    }

    @Override
    public double getTrustLevel() {
        return trustLevel;
    }
}
