package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initTeleoperated(hardwareMap);
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        Pose2d currentPose = new Pose2d();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();  // exclusive use for linear extension

        MethodReference intakeControls = () -> {
            // Reset timer on rising edge detection.
            if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) timer2.reset();
            if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) timer2.reset();

            // High edge
            if (gamepad.right_trigger > 0 && lastGamepad.right_trigger > 0) {
                if (robot.getSlider() < 300) {
                    if (robot.intakeUp) robot.setSliderPower(1);
                    else {
                        robot.intakeUp();
                        robot.intakeUp = timer2.milliseconds() > 250;  // Falsely override boolean
                    }
                } else robot.setSliderPower(1);
            }

            if (gamepad.left_trigger > 0 && lastGamepad.left_trigger > 0) {
                if (robot.getSlider() < 300) {
                    if (robot.intakeUp) robot.setSliderPower(-1);
                    else {
                        robot.intakeUp();
                        robot.intakeUp = timer2.milliseconds() > 250;  // Falsely override boolean
                    }
                } else robot.setSliderPower(-1);
            }

            // Falling edge
            if (!(gamepad.right_trigger > 0) && lastGamepad.right_trigger > 0) {
                robot.setSliderPower(0);
                robot.setSlider(robot.getSlider());
                timer2.reset();
            }

            // Lower edge
            if (!(gamepad.right_trigger > 0) && !(lastGamepad.right_trigger > 0)
                    || !(gamepad.left_trigger > 0) && !(lastGamepad.left_trigger > 0)) {
                if (robot.getSlider() > 300) {
                    if (timer2.milliseconds() > 250) robot.intakeDown();
                    robot.setSlider(robot.getSlider());
                } else {
                    robot.setSlider(0);
                    robot.intakeUp();

                    if (robot.sliderInPosition(10) && timer2.milliseconds() > 250)
                        robot.intakeDown();
                }
            }

            // General intake controls
            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (robot.clawIntakeOpen) robot.clawIntakeClose(); else robot.clawIntakeOpen();
            }
        };

        waitForStart();
        drivetrain.setPoseEstimate(Storage.lastPose);

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            currentPose = drivetrain.getPoseEstimate();

            if (state == State.INIT) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE) {
                intakeControls.call();
                
            }
        }

        robot.copyToStorage(currentPose, Project1Hardware.Operation.TELEOPERATED);
    }

    enum State {
        AUTONOMOUS_END,
        INIT,
        INTAKE,
        TRANSFER,
        AWAIT,
        TRANSITION,
        SCORE,
        RETURN
    }

    interface MethodReference {void call();}
}
