package frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;

public class VisionConstants {
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
}
