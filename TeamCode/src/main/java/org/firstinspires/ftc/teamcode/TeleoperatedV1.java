package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        boolean reversing = false;
        int chamberStep = 0;

        MethodReference intakeControls = () -> {
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
            double horizontal = gamepad.left_stick_x;
            double vertical = -gamepad.left_stick_y;
            double pivot = gamepad.right_stick_x;
            double heading = robot.getIMU();

            if (state == State.INIT) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.INTAKE;
                    timer1.reset();
                    robot.intakeDown();
                }
            }

            else if (state == State.INTAKE) {
                robot.powerOffArm();
                robot.setSlider(Storage.SLIDER_TRANSFER);
                robot.clawScoringOpen();
                robot.intakeDown();
                intakeControls.call();

                if (gamepad.triangle && !lastGamepad.triangle) {
                    state = State.TRANSFER;
                    timer1.reset();
                    timer2.reset();
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    state = State.INTAKE_EXTEND;
                    timer1.reset();
                    timer2.reset();
                }
            }

            else if (state == State.INTAKE_EXTEND) {
                intakeControls.call();

                if (gamepad.right_trigger > 0) {
                    if (robot.getSlider() > 1300) {
                        // Set power to a percentage with respect to remaining length, cubed
                        // Retain +/- power after exponent to help with overshoot
                        double d = 1590 - robot.getSlider();
                        double power = Math.abs(Math.pow(d / 1590, 3)) * (Math.abs(d) / d);
                        robot.setSliderPower(power);
                    } else robot.setSlider(1);
                } else if (gamepad.left_trigger > 0) {
                    robot.setSliderPower(-1);
                } else {
                    robot.setSlider(robot.getSlider(), 1);
                }

                if (robot.getSlider() < Storage.SLIDER_MINIMUM
                        && !(gamepad.right_trigger > 0)
                        && lastGamepad.left_trigger > 0) {
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.INTAKE;
                }

                if (gamepad.left_bumper || gamepad.triangle) {
                    robot.intakeUp();
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    if (gamepad.left_bumper) state = State.INTAKE;
                    else if (gamepad.triangle) {
                        state = State.TRANSFER;
                        timer1.reset();
                        timer2.reset();
                    }
                }
            }

            else if (state == State.TRANSFER) {
                robot.powerOffArm();

                if (robot.getSlider() >= Storage.SLIDER_MINIMUM) {
                    if (robot.intakeUp) {
                        // Intake is put down; retract slider
                        robot.setSlider(Storage.SLIDER_TRANSFER);
                    } else {
                        // Intake is put down; raise and delay before retracting
                        robot.intakeUp();
                        robot.intakeUp = timer1.milliseconds() > 500;
                    }
                } else {
                    robot.setSlider(Storage.SLIDER_TRANSFER);

                    if (!robot.intakeUp) {
                        // Intake is put down; raise and delay before retracting
                        robot.intakeUp();
                        robot.intakeUp = timer1.milliseconds() > 500;
                    }

                    if (robot.sliderInPosition(30) && robot.intakeUp) {
                        state = State.AWAIT;
                        timer1.reset();
                        robot.powerOffArm();
                        robot.clawScoringClose();
                    }
                }
            }

            else if (state == State.AWAIT) {
                if (timer1.milliseconds() > 500 && robot.sliderInPosition()) {
                    robot.clawIntakeOpen();

                    if (robot.mode == Project1Hardware.Mode.CHAMBER && timer1.milliseconds() > 750)
                        robot.setSlider(Storage.SLIDER_CLEARANCE);

                    if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                        state = State.TRANSITION;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper) {
                    robot.intakeDown();
                    robot.clawScoringOpen();
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.TRANSITION) {
                if (!reversing) {
                    switch (robot.mode) {
                        case BASKET:
                            if (-135 < robot.getIMUDegrees() && robot.getIMUDegrees() < 45)
                                robot.setArmBasketFront();
                            else robot.setArmBasketRear();
                            break;

                        case CHAMBER:
                            robot.setArmScoring();
                            break;
                    }

                    if (robot.armInPosition()) {
                        switch (robot.mode) {
                            case BASKET: state = State.SCORE_BASKET; break;
                            case CHAMBER: state = State.SCORE_CHAMBER; break;
                        }
                        timer1.reset();
                        chamberStep = 0;
                    }
                } else {
                    robot.setArmTransfer();

                    if (robot.armInPosition(30)) {
                        robot.powerOffArm();
                        state = State.AWAIT;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) reversing = true;
                if (gamepad.right_bumper && !lastGamepad.right_bumper) reversing = false;
            }

            else if (state == State.SCORE_BASKET) {
                reversing = false;

                if (-135 < robot.getIMUDegrees() && robot.getIMUDegrees() < 45)
                    robot.setArmBasketFront();
                else robot.setArmBasketRear();

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    if (robot.clawScoringOpen) robot.clawScoringClose();
                    else robot.clawScoringOpen();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION;
                    reversing = true;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.RETURN;
                    timer1.reset();
                }

                if ((gamepad.options && !lastGamepad.options)
                        || (gamepad.share && !lastGamepad.share)) {
                    state = State.TRANSITION;
                    timer1.reset();
                }
            }

            else if (state == State.SCORE_CHAMBER) {
                reversing = false;

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    chamberStep++;
                    chamberStep = Range.clip(chamberStep, 0, 2);
                }

                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    chamberStep--;
                    chamberStep = Range.clip(chamberStep, 0, 2);
                }

                switch (chamberStep) {
                    case 0: robot.setArmScoring(); robot.clawScoringClose(); break;
                    case 1: robot.setArmChambered(); robot.clawScoringClose(); break;
                    case 2: robot.setArmChambered(); robot.clawScoringOpen(); break;
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION;
                    reversing = true;
                    timer1.reset();
                }

                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.RETURN;
                    timer1.reset();
                }

                if ((gamepad.options && !lastGamepad.options)
                        || (gamepad.share && !lastGamepad.share)) {
                    state = State.TRANSITION;
                    timer1.reset();
                }
            }

            else if (state == State.RETURN) {
                robot.clawScoringOpen();
                robot.setArmTransfer();

                if (robot.armInPosition(30)) {
                    robot.powerOffArm();
                    state = State.INTAKE;
                    timer1.reset();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION;
                    reversing = true;
                    timer1.reset();
                }
            }

            if (gamepad.share) robot.mode = Project1Hardware.Mode.BASKET;
            if (gamepad.options) robot.mode = Project1Hardware.Mode.CHAMBER;
            if (gamepad.touchpad) robot.imu.resetYaw();

            robot.drivetrain.remote(vertical, horizontal, -pivot, heading);

            telemetry.addData("State", state);
            telemetry.addLine();
            telemetry.addData("Heading", robot.getIMUDegrees());
            telemetry.addData("Slider", robot.getSlider());
            telemetry.addData("Arm(E)", robot.arm.getCurrentPosition());
            telemetry.addData("Arm(A)", robot.getArmAngle());
            telemetry.update();
        }

        robot.copyToStorage(currentPose, Project1Hardware.Operation.TELEOPERATED);
    }

    enum State {
        AUTONOMOUS_END,
        INIT,
        INTAKE,
        INTAKE_EXTEND,
        TRANSFER,
        AWAIT,
        TRANSITION,
        SCORE_BASKET,
        SCORE_CHAMBER,
        RETURN
    }

    interface MethodReference {void call();}
}
