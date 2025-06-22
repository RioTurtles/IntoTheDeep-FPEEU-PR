package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initTeleoperated(hardwareMap);
        SampleMecanumDrive drivetrain = new SampleMecanumDrive(hardwareMap);
        PIDController headingController = new PIDController(0.55, 0.001, 0);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        Pose2d currentPose = new Pose2d();
        Pose2d intakePose = new Pose2d();
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();  // exclusive use for linear extension
        ElapsedTime loopTime = new ElapsedTime();

        Double autoAlignTarget;
        AtomicInteger orientationPreset = new AtomicInteger();
        boolean reversing = false, chambered = false;

        MethodReference intakeControls = () -> {
            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (robot.clawIntakeOpen) robot.clawIntakeClose(); else robot.clawIntakeOpen();
            }

            if (gamepad.circle) robot.intakeUp();
            if (gamepad.cross) robot.intakeDown();

            if (operator.dpad_right) {
                orientationPreset.getAndIncrement();
                orientationPreset.set(Range.clip(orientationPreset.intValue(), -2, 2));
            }

            if (operator.dpad_left) {
                orientationPreset.getAndDecrement();
                orientationPreset.set(Range.clip(orientationPreset.intValue(), -2, 2));
            }

            switch (orientationPreset.intValue()) {
                case -2: robot.differential.setOrientation(-90); break;
                case -1: robot.differential.setOrientation(-45); break;
                case 0: robot.differential.setOrientation(0); break;
                case 1: robot.differential.setOrientation(45); break;
                case 2: robot.differential.setOrientation(90); break;
            }
        };

        waitForStart();
        drivetrain.setPoseEstimate(Storage.lastPose);

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            currentPose = drivetrain.getPoseEstimate();
            intakePose = PoseEstimation.getIntakePose(currentPose, robot.getSliderInches());
            double horizontal = -gamepad.left_stick_x;
            double vertical = gamepad.left_stick_y;
            double pivot = -gamepad.right_stick_x;
            double heading = robot.getIMU();

            if (state == State.INIT) {
                if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                    state = State.INTAKE;
                    robot.clawIntakeOpen();
                    robot.intakeDown();
                    timer1.reset();
                }
            }

            else if (state == State.INTAKE) {
                robot.powerOffArm();
                robot.setSlider(Storage.SLIDER_TRANSFER);
                robot.clawScoringOpen();
                intakeControls.call();

                if (gamepad.triangle && !lastGamepad.triangle) {
                    state = State.TRANSFER;
                    timer1.reset();
                    timer2.reset();
                }

                if (gamepad.right_trigger > 0 && !(lastGamepad.right_trigger > 0)) {
                    state = State.INTAKE_EXTEND;
                    reversing = false;
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
                    } else robot.setSliderPower(1);
                } else if (gamepad.left_trigger > 0) {
                    robot.setSliderPower(-1);
                } else if (!reversing) {
                    robot.setSlider(robot.getSlider(), 1);
                }

                if (robot.getSlider() < Storage.SLIDER_MINIMUM
                        && !(gamepad.right_trigger > 0)
                        && lastGamepad.left_trigger > 0) {
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.INTAKE;
                }

                if (PoseEstimation.collideWithBarrier(currentPose, robot.getSliderInches()))
                    robot.intakeUp();
                else if (lastGamepad.right_trigger > 0) robot.intakeDown();

                if (gamepad.left_bumper || gamepad.triangle) {
                    if ((gamepad.left_bumper && !lastGamepad.left_bumper && !gamepad.triangle)
                            || (gamepad.triangle && !lastGamepad.triangle && !gamepad.left_bumper))
                        timer2.reset();

                    if (robot.intakeUp) {
                        robot.setSlider(Storage.SLIDER_TRANSFER);
                        if (gamepad.left_bumper) {
                            reversing = true;
                            if (robot.sliderInPosition(50)) {
                                state = State.INTAKE;
                                robot.intakeDown();
                            }
                        } else if (gamepad.triangle) {
                            state = State.TRANSFER;
                            timer1.reset();
                            timer2.reset();
                        }
                    } else {
                        robot.intakeUp();
                        robot.intakeUp = timer2.milliseconds() > 250;
                    }
                }
            }

            else if (state == State.TRANSFER) {
                reversing = false;
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
                if (timer1.milliseconds() > 500 && robot.sliderInPosition() && !reversing) {
                    robot.clawIntakeOpen();

                    if (timer1.milliseconds() > 750) {
                        robot.setSlider(Storage.SLIDER_CLEARANCE);
                        state = State.TRANSITION;
                        timer1.reset();
                    }
                }

                if (gamepad.left_bumper || reversing) {
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
                        chambered = false;
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

                if (gamepad.right_trigger > 0) {
                    chambered = true;
                    if (!(lastGamepad.right_trigger > 0)) timer1.reset();
                }

                if (gamepad.left_trigger > 0) chambered = false;

                if (chambered) {
                    robot.setArmChambered();
                    if (timer1.milliseconds() > 300) robot.clawScoringOpen();
                } else {
                    robot.setArmScoring();
                    robot.clawScoringClose();
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
                    robot.intakeDown();
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.INTAKE;
                    timer1.reset();
                }

                if (gamepad.left_bumper && !lastGamepad.left_bumper) {
                    state = State.TRANSITION;
                    reversing = true;
                    timer1.reset();
                }
            }

            if (operator.triangle) autoAlignTarget = 0.0;
            else if (operator.square) autoAlignTarget = -90.0;
            else if (operator.circle) autoAlignTarget = 90.0;
            else if (operator.cross) autoAlignTarget = 45.0;
            else autoAlignTarget = null;

            if (Objects.nonNull(autoAlignTarget)) {
                assert autoAlignTarget != null;

                double current = Math.toDegrees(heading);
                double smallerAngle = Math.min(
                        Math.abs(current - autoAlignTarget),
                        360 - Math.abs(current - autoAlignTarget)
                );

                double resultant1 = current - smallerAngle;
                if (resultant1 <= -180) resultant1 += 360;
                double resultant2 = current + smallerAngle;
                if (resultant2 > 180) resultant2 -= 360;

                if (resultant1 == autoAlignTarget) pivot = Math.toRadians(smallerAngle);
                else if (resultant2 == autoAlignTarget) pivot = Math.toRadians(-smallerAngle);

                pivot = headingController.calculate(0, pivot, loopTime.seconds());

                heading = 0;
                vertical *= 0.8;
                horizontal *= 0.8;
            } else headingController.reset();

            if (gamepad.share) robot.mode = Project1Hardware.Mode.BASKET;
            if (gamepad.options) robot.mode = Project1Hardware.Mode.CHAMBER;
            if (gamepad.touchpad) robot.imu.resetYaw();

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            drivetrain.update();

            telemetry.addData("State", state);
            telemetry.addLine();
            telemetry.addData("Pose", currentPose);
            telemetry.addData("Intake", intakePose);
            telemetry.addData("Slider(E)", robot.getSlider());
            telemetry.addData("Slider(I)", robot.getSliderInches());
            telemetry.addData("Arm(E)", robot.arm.getCurrentPosition());
            telemetry.addData("Arm(A)", robot.getArmAngle());
            telemetry.update();
            loopTime.reset();
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
