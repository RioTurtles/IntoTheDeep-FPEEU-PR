package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Storage {
    public static Project1Hardware robot;
    public static Pose2d lastPose;
    public static Project1Hardware.Operation lastOpMode;

    public static final int SLIDER_TRANSFER = 40;
    public static final int SLIDER_CLEARANCE = 255;
    public final static double SLIDER_MINIMUM = 461;

    static {
        lastPose = new Pose2d(8.54, -62.99, Math.toRadians(90.00));
        lastOpMode = Project1Hardware.Operation.TELEOPERATED;
    }
}
