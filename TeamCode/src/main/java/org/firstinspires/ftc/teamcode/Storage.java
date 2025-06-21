package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Storage {
    public static Project1Hardware robot;
    public static Pose2d lastPose;
    public static Project1Hardware.Operation lastOpMode;

    public static final int SLIDER_TRANSFER = 55;
    public static final int SLIDER_CLEARANCE = 255;
    public final static double SLIDER_MINIMUM = 461;

    static {
        lastPose = new Pose2d();
        lastOpMode = Project1Hardware.Operation.TELEOPERATED;
    }
}
