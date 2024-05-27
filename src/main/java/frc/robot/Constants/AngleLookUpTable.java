// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Key is distance in meters, Value is angle in Degrees*/
public class AngleLookUpTable {
    public static final InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap(); 
    public static final Pose3d speakerAimingPoint = new Pose3d();
    public static final ArrayList<Double> speakerTags = new ArrayList<>(Arrays.asList(3.0,7.0));

    public static void initLookuptable(){
        map.put(0.25, 1.0);
        map.put(0.5, 1.0);
        map.put(0.75, 1.0);
        map.put(1.0, 1.0);
        map.put(1.25, 1.0);
        map.put(1.5, 1.0);
        map.put(1.75, 1.0);
        map.put(2.0, 1.0);
        map.put(2.25, 1.0);
        map.put(2.5, 1.0);
        map.put(2.75, 1.0);
        map.put(3.0, 1.0);
        map.put(3.25, 1.0);
        map.put(3.5, 1.0);
        map.put(3.75, 1.0);
        map.put(4.00, 1.0);
        map.put(4.25, 1.0);
        map.put(4.5, 1.0);
        map.put(4.75, 1.0);
        map.put(5.0, 1.0);
        map.put(5.25, 1.0);
        map.put(5.5, 1.0);
        map.put(5.75, 1.0);
        map.put(6.0, 1.0);
    }
}
