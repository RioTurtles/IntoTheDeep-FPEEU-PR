package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

        Pose2d currentPose;
        Pose2d startPose = new Pose2d(8.17, -62.99, Math.toRadians(90.00));
        Pose2d chamberPose = new Pose2d(0.00, -33.89, Math.toRadians(90.00));

        TrajectorySequence pathPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(chamberPose.vec())
                .addTemporalMarker(0.25, () -> robot.setSlider(155))
                .addTemporalMarker(0.5, () ->robot.setArmAngle(30))
                .build();

        TrajectorySequence pathTransition = null;
        TrajectorySequence pathGrab2 = null;
        TrajectorySequence pathGrab3 = null;
        TrajectorySequence pathApproach = null;

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
                            .lineToSplineHeading(new Pose2d(30.00, -55.15, Math.toRadians(135.00)))
                            .build()
                    );
                    robot.clawIntakeOpen();
                    objective = Objective.TRANSITION;
                    timer1.reset();
                    run1Async.set(false);
                    run2Async.set(false);
                    run3Async.set(false);
                }
            }

            if (objective == Objective.TRANSITION) {
//                drive.followTrajectorySequence(pathApproach);
//                objective = Objective.CYCLE;
            }

            if (objective == Objective.CYCLE) {

            }

            drive.update();
            telemetry.addData("Objective", objective);
            telemetry.addData("Pose", drive.getPoseEstimate());
            telemetry.addData("Arm angle", robot.getArmAngle());
            telemetry.update();
        }
    }

    enum Objective {
        INIT,
        PATH_TO_PRELOAD,
        SCORE_PRELOAD,
        PATH_TO_FIRST,
        GRAB,
        GRAB_2,
        RETRACT_2,
        GRAB_3,
        TRANSITION,
        CYCLE,
        PARK
    }
}
