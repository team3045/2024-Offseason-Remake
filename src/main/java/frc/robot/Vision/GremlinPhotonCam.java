// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class GremlinPhotonCam implements CameraBase{
    private String name;
    private Transform3d robotToCam;
    private Vector<N3> lastStdDevs;
    private double lastTimestamp;
    private SimCameraProperties properties;
    private double trustLevel;
    private Pose2d currPose;

    private PhotonCamera myCam;

    private PhotonPoseEstimator mPhotonPoseEstimator = new PhotonPoseEstimator(
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
        PoseStrategy.AVERAGE_BEST_TARGETS, 
        new Transform3d());

    public GremlinPhotonCam(String name, Transform3d robotToCam, double trustLevel, SimCameraProperties properties){
        this.name = name;
        this.robotToCam = robotToCam;
        this.trustLevel = trustLevel;
        this.properties = properties;

        myCam = new PhotonCamera(name);

        lastStdDevs = CameraBase.baseVisionStdDev;
        lastTimestamp = Timer.getFPGATimestamp();

        mPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        mPhotonPoseEstimator.setRobotToCameraTransform(robotToCam);
    }

    @Override
    public Vector<N3> getVisionStdDevs() {
        return lastStdDevs;
    }

    @Override
    public String getName() {
        return name;
    }

    //TODO: Look into comparing best and alt transforms
    @Override
    public Optional<Pose3d> getEstimatedPose3d() {
        mPhotonPoseEstimator.setReferencePose(currPose); //Only used if PoseStrategy.CLOSES_TO_REFERENCE
        var result = myCam.getLatestResult();
        lastTimestamp = result.getTimestampSeconds(); //update timestamp

        var estimate = mPhotonPoseEstimator.update(result);

        if(estimate.isEmpty()){
            return Optional.empty();
        }

        Pose3d pose = estimate.get().estimatedPose;
        PoseStrategy strat = estimate.get().strategy;
        lastTimestamp = estimate.get().timestampSeconds;

        lastStdDevs = calcStdDevs(estimate.get());

        if(lastStdDevs.elementSum() > CameraBase.throwoutStdDev.elementSum() - 1){
            return Optional.empty();
        }

        return Optional.of(pose);      
    }

    @Override
    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    @Override
    public double getTimeStamp() {
        return lastTimestamp;
    }

    @Override
    public SimCameraProperties getCamProperties() {
        return properties;
    }

    public Vector<N3> calcStdDevs(EstimatedRobotPose estimate){
        double xyStd = VisionConstants.baseStdX;

        double avgDist = 0;
        double avgAmb = 0;
        double avgArea = 0;
        boolean trustedTags = false;

        /*Calc averages and check for trusted tags */
        for(PhotonTrackedTarget target : estimate.targetsUsed){
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            avgAmb += target.getPoseAmbiguity();
            avgArea += target.getArea();

            if(VisionConstants.trustedTags.contains(target.getFiducialId())){
                trustedTags = true;
            }
        }

        avgDist /= estimate.targetsUsed.size();
        avgAmb /= estimate.targetsUsed.size();
        avgArea /= estimate.targetsUsed.size();

        if(avgDist >= VisionConstants.throwoutDistanceMeters ||
            avgAmb >= VisionConstants.throwoutAmbiguity){
            return CameraBase.throwoutStdDev;
        }

        if(
            estimate.estimatedPose.getTranslation().getX() < VisionConstants.minFieldX ||
            estimate.estimatedPose.getTranslation().getX() > VisionConstants.maxFieldX ||
            estimate.estimatedPose.getTranslation().getY() < VisionConstants.minFieldY ||
            estimate.estimatedPose.getTranslation().getY() > VisionConstants.maxFieldY
        ){
            return CameraBase.throwoutStdDev;
        }

        xyStd *= trustLevel; //Trust one cam more than others, Works on the Base
        if(avgDist >= VisionConstants.trustedMaxDistance){
            double addedStd = VisionConstants.distanceModifier * avgDist;
        }

         /*Trust measurements with multiple tags more */
         if(estimate.targetsUsed.size() > 1){
            xyStd /= VisionConstants.multTagDivisor;
        }

        /*We trust some tags more than others*/
        if(trustedTags){
            xyStd *= VisionConstants.trustedTagModifier;
        }

        /*Increases stdDev if more ambiguous*/
        if(avgAmb > VisionConstants.trustedMaxAmbiguity){
            xyStd *= (1+avgAmb);
        }

        return VecBuilder.fill(xyStd, xyStd, Units.degreesToRadians(90000));


    }

    public void updateCurrPose(Pose2d update){
        currPose = update;
    }

    @Override
    public double getTrustLevel() {
        return trustLevel;
    }
    
    public PhotonCamera getPhotonCamera(){
        return myCam;
    }
    
}
