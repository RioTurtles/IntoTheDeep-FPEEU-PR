package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Storage {
    public static Project1Hardware robot;
    public static Pose2d lastPose;
    public static Project1Hardware.Operation lastOpMode;

    public final static double REFACTOR_CONSTANT = 384.5 / 145.1;
    public static final int SLIDER_TRANSFER = (int) Math.round(40 / REFACTOR_CONSTANT);
    public static final int SLIDER_CLEARANCE = (int) Math.round(255 / REFACTOR_CONSTANT);
    public final static int SLIDER_MINIMUM = (int) Math.round(461 / REFACTOR_CONSTANT);

    static {
        lastPose = new Pose2d(8.54, -62.99, Math.toRadians(90.00));
        lastOpMode = Project1Hardware.Operation.TELEOPERATED;
    }
}
