package frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static final boolean USE_MULT_TAG = true;

    public static final double throwoutDistanceMeters = 6;
    public static final double throwoutAmbiguity = 0.4;
    public static final double minFieldX = 0;
    public static final double maxFieldX = 40;
    public static final double minFieldY = 0;
    public static final double maxFieldY = 20;
    public static final double multTagDivisor = 2;
    public static final double trustedTagModifier = 0.8;
    public static final double distanceModifier = 0.3;
    public static final double trustedMaxDistance = 2;
    public static final double trustedMaxAmbiguity = 0.2;

    public static final double baseStdX = 5;
    public static final double baseStdY = 5;
    public static final double baseStdDeg = 9000;

    public static final ArrayList<Integer> trustedTags = new ArrayList<>(Arrays.asList(6,7));

    public static final double frontCamTrust = 1;
    public static final double intakeCamTrust = 1;
    
    /*Robot to Camera tranforms */
    public static final Transform3d frontTransform = new Transform3d();
    public static final Transform3d intakeTransform = new Transform3d();


    /*Camera Calibration Properties form simulation,
    * Can customize for each camera but not rly necessary */
    //TODO: Change to actual LL3 Properties
    public static SimCameraProperties LL3_1280_720(){
        var prop = new SimCameraProperties();
        prop.setCalibration(
                1280,
                720,
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N3(),
                        // intrinsic
                        1011.3749416937393,
                        0.0,
                        645.4955139388737,
                        0.0,
                        1008.5391755084075,
                        508.32877656020196,
                        0.0,
                        0.0,
                        1.0),
                VecBuilder.fill( // distort
                        0.13730101577061535,
                        -0.2904345656989261,
                        8.32475714507539E-4,
                        -3.694397782014239E-4,
                        0.09487962227027584));
        prop.setCalibError(0.37, 0.06);
        prop.setFPS(30);
        prop.setAvgLatencyMs(50);
        prop.setLatencyStdDevMs(20);
        return prop;
    }
}
