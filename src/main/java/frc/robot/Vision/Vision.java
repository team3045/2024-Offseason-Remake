package frc.robot.Vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public interface Vision {
    final Vector<N3> baseVisionStdDev = VecBuilder.fill(5, 5, Units.degreesToRadians(900));
    final Vector<N3> throwoutStdDev = VecBuilder.fill(100, 100, Units.degreesToRadians(90000));

    Vector<N3> getVisionStdDevs();
    String getName();
    Optional<Pose3d> getEstimatedPose3d();
    Transform3d getRobotToCam();
    double getTimeStamp();
}
