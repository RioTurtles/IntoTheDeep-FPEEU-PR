package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@TeleOp(name="TeleoperatedV2")
public class Inspection extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initTeleoperated(hardwareMap);
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        TeleoperatedV1.State state = TeleoperatedV1.State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();

        Pose2d currentPose;
        Pose2d intakePose;
        int cycles = 0;

        waitForStart();
        drive.setPoseEstimate(new Pose2d(8.54, -62.99, Math.toRadians(90.00)));
        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();
            intakePose = PoseEstimation.getIntakePose(currentPose, robot.getSliderInches());
            lastGamepad.copy(gamepad);
            gamepad.copy(gamepad1);

            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (robot.getArmAngle() < 10) {robot.setArmAngle(125);
                    state = TeleoperatedV1.State.SCORING;}
                else robot.setArmTransfer();
            }

            if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                if (robot.getSlider() < 100) robot.setSlider(550);
                else robot.setSlider(Storage.SLIDER_TRANSFER);
                state = TeleoperatedV1.State.INTAKE_EXTEND;
                robot.intakeUp2();
            }

            drive.update();
            robot.drivetrain.remote(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x, robot.getIMU());
            telemetry.addData("State", state);
            telemetry.addData("Mode", robot.mode);
            telemetry.addData("Cycles", cycles);
            telemetry.addLine();
            telemetry.addData("Pose", currentPose);
            telemetry.addData("Intake", intakePose);
            telemetry.addData("Slider(E)", robot.getSlider());
            telemetry.addData("Slider(I)", robot.getSliderInches());
            telemetry.addData("Arm(E)", robot.arm.getCurrentPosition());
            telemetry.addData("Arm(A)", robot.getArmAngle());
            telemetry.update();
        }
    }
}
