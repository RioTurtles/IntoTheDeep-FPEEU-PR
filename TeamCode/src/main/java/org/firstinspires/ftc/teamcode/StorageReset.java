package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Clear Autonomous Data")
public class StorageReset extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initUndefined(hardwareMap);
        final Pose2d pose = new Pose2d(8.54, -62.99, Math.toRadians(90.00));
        final Project1Hardware.Operation operation = Project1Hardware.Operation.TELEOPERATED;

        robot.resetEncoders();
        robot.copyToStorage(pose, operation);
        robot.imu.resetYaw();
        Storage.lastPose = pose;
        Storage.lastOpMode = operation;

        terminateOpModeNow();
    }
}
