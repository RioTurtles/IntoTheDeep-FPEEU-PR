package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseEstimation {
    public static final double ROBOT_TO_INTAKE = 185.07463 / 25.4;

    public static Pose2d getIntakePose(Pose2d robotPose, double sliderInches) {
        double heading = robotPose.getHeading() - Math.toRadians(90.00);
        double k = ROBOT_TO_INTAKE + sliderInches;
        double x = robotPose.getX() + k * Math.sin(heading);
        double y = robotPose.getY() - k * Math.cos(heading);
        return new Pose2d(x, y, heading);
    }

    public static boolean collideWithBarrier(Pose2d robotPose, double sliderInches) {
        Pose2d intakePose = getIntakePose(robotPose, sliderInches);
        boolean longEdge = (((9.454 <= intakePose.getX() && intakePose.getX() <= 23.454)
                || (-23.454 <= intakePose.getX() && intakePose.getX() <= -9.454))
                && (-22.25 <= intakePose.getY() && intakePose.getY() <= 22.5));
        boolean shortEdge = (((-16.05 <= intakePose.getX() && intakePose.getX() <= 16.05)
                || (-25.20 <= intakePose.getY() && intakePose.getY() <= -23.20))
                && (23.20 <= intakePose.getY() && intakePose.getY() <= 23.20));

        return longEdge || shortEdge;
    }

    public static boolean intakeInObservation(Pose2d robotPose, double sliderInches) {
        Pose2d intakePose = getIntakePose(robotPose, sliderInches);
        boolean rectangle = 48.24 <= intakePose.getX() && intakePose.getY() <= -59.20;
        boolean triangle = 40.315 <= intakePose.getX()
                && intakePose.getY() <= -70.125 + Math.abs(intakePose.getX() - 10.925);

        return rectangle || triangle;
    }
}
