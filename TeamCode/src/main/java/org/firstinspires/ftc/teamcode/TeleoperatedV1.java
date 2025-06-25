package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@TeleOp
public class TeleoperatedV1 extends LinearOpMode {
    Trajectory pathGrab = null;
    Trajectory pathChamber = null;
    Trajectory pathObservation = null;
    int cycles = 0;

    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initTeleoperated(hardwareMap);
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        Project1Hardware.PIDController headingController = new Project1Hardware.PIDController(0.55, 0.001, 0);
        State state = State.INIT;
        Gamepad gamepad = new Gamepad();
        Gamepad lastGamepad = new Gamepad();
        Gamepad operator = new Gamepad();
        Gamepad lastOperator = new Gamepad();

        Pose2d currentPose = new Pose2d();
        Pose2d intakePose;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();  // exclusive use for linear extension
        ElapsedTime loopTime = new ElapsedTime();

        Double autoAlignTarget;
        AtomicInteger orientationPreset = new AtomicInteger();
        AtomicBoolean pathingComplete = new AtomicBoolean(false);
        boolean reversing = false, chambered = false;
        boolean retractionInitiated = false;

        MethodReference intakeControls = () -> {
            if (gamepad.right_bumper && !lastGamepad.right_bumper) {
                if (robot.clawIntakeOpen) robot.clawIntakeClose(); else robot.clawIntakeOpen();
            }

            if (gamepad.circle) robot.intakeUp();
            if (gamepad.cross) robot.intakeDown();

            if (operator.dpad_right && !lastOperator.dpad_right) {
                orientationPreset.getAndIncrement();
                orientationPreset.set(Range.clip(orientationPreset.intValue(), -2, 2));
            }

            if (operator.dpad_left && !lastOperator.dpad_left) {
                orientationPreset.getAndDecrement();
                orientationPreset.set(Range.clip(orientationPreset.intValue(), -2, 2));
            }

            switch (orientationPreset.intValue()) {
                case -2: case 2: robot.differential.setOrientation(-90); break;
                case -1: robot.differential.setOrientation(-45); break;
                case 0: robot.differential.setOrientation(0); break;
                case 1: robot.differential.setOrientation(45); break;
            }
        };

        waitForStart();
        drive.setPoseEstimate(Storage.lastPose);

        if (Storage.lastOpMode == Project1Hardware.Operation.AUTONOMOUS)
            state = State.AUTONOMOUS_END;

        while (opModeIsActive()) {
            lastGamepad.copy(gamepad); gamepad.copy(gamepad1);
            lastOperator.copy(operator); operator.copy(gamepad2);
            currentPose = drive.getPoseEstimate();
            intakePose = PoseEstimation.getIntakePose(currentPose, robot.getSliderInches());
            double horizontal = -gamepad.left_stick_x;
            double vertical = gamepad.left_stick_y;
            double pivot = -gamepad.right_stick_x;
            double heading = robot.getIMU();

            if (state == State.AUTONOMOUS_END) {
                state = State.INTAKE_EXTEND;
                timer1.reset();
            }

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
                retractionInitiated = false;

                if (gamepad.left_trigger > 0 && !(lastGamepad.left_trigger > 0)) {
                    state = State.PASSING;
                    timer1.reset();
                }

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

                if (gamepad.right_trigger > 0 || operator.right_trigger > 0) {
                    retractionInitiated = false;
                    double power;
                    if (robot.getSlider() > 500) {
                        double d = 590 - robot.getSlider();
                        power = Math.abs(Math.pow(d / 590, 3)) * (Math.abs(d) / d);
                    } else power = 1.0;

                    if (gamepad.right_trigger > 0) robot.setSliderPower(power * 0.7);
                    else if (operator.right_trigger > 0) robot.setSliderPower(power * 0.2);

                } else if (gamepad.left_trigger > 0 || operator.left_trigger > 0) {
                    if (gamepad.left_trigger > 0 && lastGamepad.left_trigger > 0)
                        robot.setSliderPower(-0.7);
                    else if (operator.left_trigger > 0 && lastOperator.left_trigger > 0)
                        robot.setSliderPower(-0.2);

                } else if (!retractionInitiated) robot.setSliderPower(0);

                if (robot.getSlider() < Storage.SLIDER_MINIMUM
                        && !(gamepad.right_trigger > 0)
                        && lastGamepad.left_trigger > 0) {
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.INTAKE;
                }

                if (PoseEstimation.collideWithBarrier(currentPose, robot.getSliderInches())
                        && !gamepad.cross)
                    robot.intakeUp();
                else if (lastGamepad.right_trigger > 0) robot.intakeDown();

                if (gamepad.left_bumper || gamepad.triangle) {
                    retractionInitiated = true;
                    timer1.reset();
                    timer2.reset();
                }

                if (retractionInitiated) {
                    if (robot.intakeUp) {
                        robot.setSlider(Storage.SLIDER_TRANSFER);
                        if (robot.mode == Project1Hardware.Mode.SAMPLE) {
                            if (robot.sliderInPosition(50)) {
                                state = State.INTAKE;
                                if (!PoseEstimation.collideWithBarrier(
                                        currentPose,
                                        robot.getSliderInches()
                                )) robot.intakeDown();
                                timer1.reset();
                                timer2.reset();
                            }
                        } else if (robot.mode == Project1Hardware.Mode.SPECIMEN) {
                            if (robot.sliderInPosition(50)) {
                                state = State.TRANSFER;
                                robot.intakeUp();
                                timer1.reset();
                                timer2.reset();
                            }
                        }
                        robot.differential.setOrientation(0);
                    } else {
                        robot.intakeUp();
                        robot.intakeUp = timer2.milliseconds() > 250;
                    }
                }
            }

            else if (state == State.PASSING) {
                robot.intakeDown();
                robot.setSlider(575);

                if (!(gamepad.left_trigger > 0) && lastGamepad.left_trigger > 0) {
                    robot.clawIntakeOpen();
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.TRANSFER) {
                reversing = false;
                robot.differential.setOrientation(0);
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
                    robot.setArmScoring();

                    if (robot.armInPosition()) {
                        state = State.SCORING;
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

            else if (state == State.SCORING) {
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

            else if (state == State.RESET) {
                robot.intakeUp();

                if (operator.dpad_up) robot.setSliderPower(-1); else robot.setSliderPower(0);

                if (operator.dpad_down) robot.arm.setPower(-1); else robot.arm.setPower(0);

                if (operator.touchpad) {
                    state = State.INTAKE;
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.sliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.sliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

            else if (state == State.RR_PATH_GRAB) {
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(pathGrab);
                    robot.clawIntakeOpen();
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    if (robot.getSlider() > Storage.SLIDER_MINIMUM) state = State.INTAKE_EXTEND;
                    else state = State.INTAKE;
                    timer1.reset();
                }

                if (pathingComplete.get()) {
                    state = State.RR_GRAB;
                    timer1.reset();
                }
            }

            else if (state == State.RR_GRAB) {
                pathingComplete.set(false);

                if (timer1.milliseconds() > 750) {
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    state = State.RR_TRANSFER;
                    timer1.reset();
                }
                else if (timer1.milliseconds() > 500) robot.intakeUp();
                else if (timer1.milliseconds() > 250) robot.clawIntakeClose();
                else getPath2(drive, currentPose, () -> pathingComplete.set(true));

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    if (robot.getSlider() > Storage.SLIDER_MINIMUM) state = State.INTAKE_EXTEND;
                    else state = State.INTAKE;
                    timer1.reset();
                }
            }

            else if (state == State.RR_TRANSFER) {
                if (!drive.isBusy()) drive.followTrajectoryAsync(pathChamber);

                if (robot.sliderInPosition() || timer1.milliseconds() > 1000) {
                    state = State.RR_APPROACH;
                    robot.clawScoringClose();
                    timer1.reset();
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    if (robot.getSlider() > Storage.SLIDER_MINIMUM) state = State.INTAKE_EXTEND;
                    else state = State.TRANSFER;
                    timer1.reset();
                }
            }

            else if (state == State.RR_APPROACH) {
                if (pathingComplete.get()) {
                    robot.setArmAngle(135);
                    state = State.RR_SCORE;
                    timer1.reset();
                } else if (timer1.milliseconds() > 500) {
                    robot.setSlider(Storage.SLIDER_CLEARANCE);
                    robot.setArmAngle(30);
                } else if (timer1.milliseconds() > 250) robot.clawIntakeOpen();

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    if (robot.getSlider() > Storage.SLIDER_MINIMUM) state = State.INTAKE_EXTEND;
                    else state = State.TRANSITION;
                    timer1.reset();
                }
            }

            else if (state == State.RR_SCORE) {
                robot.intakeDown();
                robot.clawIntakeOpen();

                if (timer1.milliseconds() > 500) {
                    drive.followTrajectoryAsync(pathObservation);
                    pathingComplete.set(false);
                    state = State.RR_PATH_RETURN;
                } else if (robot.getArmAngle() > 110 || timer1.milliseconds() > 250) {
                    cycles++;
                    robot.clawScoringOpen();
                    robot.setSlider(590);
                } else getPath3(drive, currentPose, () -> pathingComplete.set(true));

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    state = State.SCORING;
                    chambered = robot.getArmAngle() >= 100;
                    timer1.reset();
                }
            }

            else if (state == State.RR_PATH_RETURN) {
                if (!drive.isBusy()) {
                    state = State.RR_GRAB;
                    timer1.reset();
                }

                if (gamepad.circle && !lastGamepad.circle) {
                    drive.breakFollowing();
                    state = State.INTAKE_EXTEND;
                    timer1.reset();
                }
            }

            if (gamepad.square && !lastGamepad.square) {
                if (robot.clawScoringOpen) {
                    getPath1(drive, currentPose, () -> {
                        robot.setSlider(590);
                        pathingComplete.set(true);
                    });
                    state = State.RR_PATH_GRAB;
                    timer1.reset();
                    pathingComplete.set(false);
                } else {
                    getPath2(drive, currentPose, () -> {
                        robot.setSlider(Storage.SLIDER_TRANSFER);
                        pathingComplete.set(true);
                    });
                    state = State.RR_APPROACH;
                    timer1.reset();
                    pathingComplete.set(false);
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

            if (operator.share) robot.mode = Project1Hardware.Mode.SAMPLE;
            if (operator.options) robot.mode = Project1Hardware.Mode.SPECIMEN;

            if (operator.dpad_up || operator.dpad_down) {
                robot.powerOffArm();
                robot.sliderLeft.setPower(0);
                robot.sliderRight.setPower(0);
                state = State.RESET;
            }

            if (gamepad.touchpad) {
                robot.imu.resetYaw();
                drive.setPoseEstimate(new Pose2d(
                        currentPose.getX(),
                        currentPose.getY(),
                        Math.toRadians(90.00)
                ));
            }

            robot.drivetrain.remote(vertical, horizontal, pivot, heading);
            drive.update();

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
        PASSING,
        TRANSFER,
        AWAIT,
        TRANSITION,
        SCORING,
        RETURN,
        RESET,
        RR_PATH_GRAB,
        RR_GRAB,
        RR_TRANSFER,
        RR_APPROACH,
        RR_SCORE,
        RR_PATH_RETURN
    }

    public void getPath1(SampleMecanumDriveCancelable drive, Pose2d pose, MarkerCallback callback) {
        TrajectoryBuilder builder = drive.trajectoryBuilder(pose);

        if (pose.getY() > -29.19) {
            if (pose.getX() > 0)
                builder.splineTo(new Vector2d(39.33, -29.19), Math.toRadians(243.21));
            else builder.splineTo(new Vector2d(-39.33, -29.19), Math.toRadians(243.21));
        }

        pathGrab = builder
                .lineToSplineHeading(new Pose2d(20.57, -46.46, Math.toRadians(135.00)))
                .addSpatialMarker(new Vector2d(20.57, -46.46), callback)
                .build();
    }

    public void getPath2(SampleMecanumDriveCancelable drive, Pose2d pose, MarkerCallback callback) {
        pathChamber = drive.trajectoryBuilder(pose)
                .lineToSplineHeading(new Pose2d(7.20 - (cycles * 1.75), -33.25, Math.toRadians(90.00)))
                .addSpatialMarker(new Vector2d(7.20 - (cycles * 1.75)), callback)
                .build();
    }

    public void getPath3(SampleMecanumDriveCancelable drive, Pose2d pose, MarkerCallback callback) {
        pathObservation = drive.trajectoryBuilder(pose)
                .lineToSplineHeading(new Pose2d(20.57, -46.46, Math.toRadians(135.00)))
                .addSpatialMarker(new Vector2d(20.57, -46.46), callback)
                .build();
    }

    interface MethodReference {void call();}
}
