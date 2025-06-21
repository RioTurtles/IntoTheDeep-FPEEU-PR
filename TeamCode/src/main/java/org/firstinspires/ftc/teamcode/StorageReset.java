package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class StorageReset extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initUndefined(hardwareMap);
        robot.resetEncoders();
        robot.copyToStorage(new Pose2d(), Project1Hardware.Operation.TELEOPERATED);
    }
}
