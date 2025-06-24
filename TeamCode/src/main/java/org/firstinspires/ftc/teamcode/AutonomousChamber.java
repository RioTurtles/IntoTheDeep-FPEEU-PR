package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class AutonomousChamber extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initAutonomous(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Objective objective = Objective.INIT;
        ElapsedTime timer1 = new ElapsedTime();
        ElapsedTime timer2 = new ElapsedTime();

        AtomicBoolean run1Async = new AtomicBoolean(false);
        AtomicBoolean run2Async = new AtomicBoolean(false);
        AtomicBoolean run3Async = new AtomicBoolean(false);
        int cycles = 0;

        Pose2d currentPose = new Pose2d();
        Pose2d startPose = new Pose2d(8.17, -62.99, Math.toRadians(90.00));
        Pose2d chamberPose = new Pose2d(0.00, -33.89, Math.toRadians(90.00));

        TrajectorySequence pathPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(chamberPose.vec())
                .addTemporalMarker(0.25, () -> robot.setSlider(155))
                .addTemporalMarker(0.5, () ->robot.setArmAngle(30))
                .build();

        TrajectorySequence pathTransition = null;
        TrajectorySequence pathCycleEnter = null;
        TrajectorySequence pathCycleExit = null;
        TrajectorySequence pathPark = null;

        robot.clawScoringClose();

        telemetry.addData("Objective", objective);
        telemetry.update();

        waitForStart();
        drive.setPoseEstimate(startPose);
        objective = Objective.PATH_TO_PRELOAD;

        while (opModeIsActive()) {
            currentPose = drive.getPoseEstimate();

            if (objective == Objective.PATH_TO_PRELOAD) {
                drive.followTrajectorySequence(pathPreload);
                objective = Objective.SCORE_PRELOAD;
                timer1.reset();
            }

            if (objective == Objective.SCORE_PRELOAD) {
                robot.setArmAngle(135);
                robot.setSlider(0);

                if (robot.getArmAngle() > 110 || timer1.milliseconds() > 250) {
                    robot.clawScoringOpen();
                    objective = Objective.PATH_TO_FIRST;
                    timer1.reset();
                }

                pathTransition = drive.trajectorySequenceBuilder(new Pose2d(-0.09, -33.89, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(31.26, -45.09, Math.toRadians(225.00)))
                        .addDisplacementMarker(1.0, () -> {
                            robot.setSlider(550);
                            robot.intakeDown();
                            robot.differential.setOrientation(-45);
                            run2Async.set(true);
                        })
                        .build();
            }

            if (objective == Objective.PATH_TO_FIRST) {
                if (!run1Async.get()) {drive.followTrajectorySequence(pathTransition); run1Async.set(true);}

                if (run2Async.get()) {
                    robot.setArmTransfer();
                    objective = Objective.GRAB;
                    run1Async.set(false);
                    run2Async.set(false);
                    run3Async.set(false);
                    timer1.reset();
                }
            }

            if (objective == Objective.GRAB) {
                // First
                robot.clawIntakeClose();
                if (!run1Async.get() && !run2Async.get() && !run3Async.get() && timer1.milliseconds() > 250) {
                    drive.turn(Math.toRadians(-100.00));
                    robot.clawIntakeOpen();
                    run1Async.set(true);
                    drive.followTrajectory(drive.trajectoryBuilder(pathTransition.end())
                            .lineToSplineHeading(new Pose2d(40.15, -43.99, Math.toRadians(225.00)))
                            .build());
                    robot.clawIntakeClose();
                    timer1.reset();
                }


                // Second
                if (run1Async.get() && !run2Async.get() && !run3Async.get() && timer1.milliseconds() > 250) {
                    drive.turn(Math.toRadians(-115.00));
                    robot.clawIntakeOpen();
                    run2Async.set(true);
                    robot.setSlider(300);
                    robot.intakeSetOrientation(-70);
                    drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(41.45, -43.99, Math.toRadians(225.00)))
                            .lineToSplineHeading(new Pose2d(54.30, -31.61, Math.toRadians(200.00)))
                            .build());
                    robot.clawIntakeClose();
                    timer1.reset();
                }

                // Third
                if (run1Async.get() && run2Async.get() && !run3Async.get() && timer1.milliseconds() > 250) {
                    robot.clawIntakeClose();
                    drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(54.30, -31.61, Math.toRadians(200.00)))
                            .addTemporalMarker(0.75, () -> {
                                robot.clawIntakeOpen();
                                robot.intakeSetOrientation(0);
                            })
                            .lineToSplineHeading(new Pose2d(27.00, -52.15, Math.toRadians(135.00)))
                            .build()
                    );
                    robot.clawIntakeOpen();
                    objective = Objective.CYCLE_INTAKE;
                    timer1.reset();
                    timer2.reset();
                    run1Async.set(false);
                    run2Async.set(false);
                    run3Async.set(false);
                }

                pathCycleEnter = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(new Pose2d(-1.21, -33.89, Math.toRadians(90.00)))
                        .build();
            }

            if (objective == Objective.CYCLE_INTAKE) {
                if (timer1.milliseconds() > 500) {
                    robot.setSlider(Storage.SLIDER_TRANSFER);
                    if (robot.sliderInPosition() || timer1.milliseconds() > 2000) {
                        robot.clawScoringClose();
                        objective = Objective.CYCLE_TRANSFER;
                        timer1.reset();
                    }
                } else if (timer1.milliseconds() > 250) {
                    robot.intakeUp();
                    if (!run1Async.get()) {drive.followTrajectorySequenceAsync(pathCycleEnter); run1Async.set(true);}
                } else {
                    pathCycleEnter = drive.trajectorySequenceBuilder(currentPose)
                            .lineToSplineHeading(new Pose2d(1.95 + (cycles * 1.8), -33.25, Math.toRadians(90.00)))
                            .build();
                    robot.clawIntakeClose();
                }
            }

            if (objective == Objective.CYCLE_TRANSFER) {
                if (timer1.milliseconds() > 750) {
                    robot.setSlider(Storage.SLIDER_CLEARANCE);

                    if (robot.sliderInPosition() || timer1.milliseconds() > 500) {
                        robot.setArmAngle(30);
                        if (robot.armInPosition() || timer1.milliseconds() > 800) {
                            objective = Objective.CYCLE_SCORE;
                            timer1.reset();
                        }
                    }
                } else if (timer1.milliseconds() > 500) robot.clawIntakeOpen();
            }

            if (objective == Objective.CYCLE_SCORE) {
                robot.intakeDown();
                robot.clawIntakeOpen();

                if (!drive.isBusy() || timer2.milliseconds() > 4000) {
                    robot.setArmAngle(135);
                    if (robot.getArmAngle() > 110 || timer1.milliseconds() > 1500) {
                        robot.clawScoringOpen();
                        objective = Objective.CYCLE_RETURN;
                        timer1.reset();
                    }
                }

                pathCycleExit = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(new Pose2d(20.57, -46.46, Math.toRadians(135.00)))
                        .addTemporalMarker(0.5, robot::setArmTransfer)
                        .build();

                pathPark = drive.trajectorySequenceBuilder(currentPose)
                        .lineToSplineHeading(new Pose2d(24.57, -46.46, Math.toRadians(135.00)))
                        .addTemporalMarker(0.5, robot::setArmTransfer)
                        .build();
            }

            if (objective == Objective.CYCLE_RETURN) {
                cycles++;
                robot.setSlider(590);

                if (cycles < 4) {
                    objective = Objective.CYCLE_DELAY;
                    drive.followTrajectorySequence(pathCycleExit);
                } else {
                    objective = Objective.PARK;
                    drive.followTrajectorySequence(pathPark);
                }

                timer1.reset();
                timer2.reset();
                run1Async.set(false);
                run2Async.set(false);
            }

            if (objective == Objective.CYCLE_DELAY) {
                if (timer1.milliseconds() > 250) {
                    objective = Objective.CYCLE_INTAKE;
                    timer1.reset();
                    timer2.reset();
                    run1Async.set(false);
                    run2Async.set(false);
                }
            }

            drive.update();
            telemetry.addData("Objective", objective);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Arm angle", robot.getArmAngle());
            telemetry.update();
        }

        robot.copyToStorage(currentPose, Project1Hardware.Operation.AUTONOMOUS);
    }

    enum Objective {
        INIT,
        PATH_TO_PRELOAD,
        SCORE_PRELOAD,
        PATH_TO_FIRST,
        GRAB,
        CYCLE_INTAKE,
        CYCLE_TRANSFER,
        CYCLE_SCORE,
        CYCLE_RETURN,
        CYCLE_DELAY,
        PARK
    }
}
