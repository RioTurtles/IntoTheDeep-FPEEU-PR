//package org.firstinspires.ftc.teamcode.commands.intake;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.RobotState.Alliance;
//import org.firstinspires.ftc.teamcode.commands.staging.GoToSampleStagingCommand;
//import org.firstinspires.ftc.teamcode.commands.staging.NoSampleStagingCommand;
//import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.EndEffectorSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
//
//public class AutoAimAndIntakeCommand extends CommandBase {
//
//    private final ArmSubsystem arm;
//    private final EndEffectorSubsystem endEffector;
//    private final VisionSubsystem vision;
//    private final DrivetrainSubsystem drivetrain;
//    private final Alliance alliance;
//
//    private enum State {
//        STARTING, SEARCHING_PRIMARY, SEARCHING_SECONDARY,
//        STRAFING_TO_ALIGN, MOVING_ARM_AND_CLAW,
//        LOWERING_CLAW, GRIPPING, LIFTING_CLAW, STAGING,
//        FAIL, FINISHED
//    }
//    private State currentState;
//    private final ElapsedTime timer = new ElapsedTime();
//
//    private double lockedSliderTarget = 0;
//    private double lockedRotationTarget = 0;
//    private double lockedHorizontalAngleGoal = 0;
//
//    private static final double STRAFE_P_GAIN = 0.025;
//    private static final double STRAFE_TOLERANCE_DEG = 1.5;
//    private static final double MAX_STRAFE_POWER = 0.4;
//    private static final double MOVEMENT_TIMEOUT_S = 0.8;
//    private static final double MIN_STRAFE_POWER = 0.25;
//
//    public AutoAimAndIntakeCommand(ArmSubsystem arm, EndEffectorSubsystem endEffector, VisionSubsystem vision, DrivetrainSubsystem drivetrain, Alliance alliance) {
//        this.arm = arm;
//        this.endEffector = endEffector;
//        this.vision = vision;
//        this.drivetrain = drivetrain;
//        this.alliance = alliance;
//        addRequirements(arm, endEffector, vision, drivetrain);
//    }
//
//    @Override
//    public void initialize() { currentState = State.STARTING; }
//
//    @Override
//    public void execute() {
//        switch (currentState) {
//            case STARTING:
//                arm.setPivotAngle(0);
//                endEffector.goToIntakePositioningState();
//                vision.setPipeline(Constants.Vision.YELLOW_PIPELINE_ID);
//                timer.reset();
//                currentState = State.SEARCHING_PRIMARY;
//                break;
//            case SEARCHING_PRIMARY:
//                if (vision.hasTarget()) {
//                    lockOntoTargetAndDecideAction();
//                } else if (timer.seconds() > 1.0) {
//                    vision.setPipeline(alliance == Alliance.BLUE ? Constants.Vision.BLUE_PIPELINE_ID : Constants.Vision.RED_PIPELINE_ID);
//                    timer.reset();
//                    currentState = State.SEARCHING_SECONDARY;
//                }
//                break;
//            case SEARCHING_SECONDARY:
//                if (vision.hasTarget()) {
//                    lockOntoTargetAndDecideAction();
//                } else if (timer.seconds() > 1.0) {
//                    currentState = State.FAIL;
//                }
//                break;
//
//            case STRAFING_TO_ALIGN:
//                if (!vision.hasTarget()) {
//                    drivetrain.stop();
//                    currentState = State.FAIL;
//                    break;
//                }
//
//                // The error is the difference between the camera's current angle and our goal angle.
//                double currentTx = vision.getTargetHorizontalAngle();
//                double strafeError = currentTx - lockedHorizontalAngleGoal;
//
//                // Check if the current angle is close enough to the goal angle.
//                if (Math.abs(strafeError) < STRAFE_TOLERANCE_DEG) {
//                    drivetrain.stop();
//                    timer.reset();
//                    currentState = State.MOVING_ARM_AND_CLAW;
//                    break;
//                }
//
//                double proportionalPower = STRAFE_P_GAIN * strafeError;
//                double strafePower;
//
//                if (Math.abs(proportionalPower) < MIN_STRAFE_POWER) {
//                    strafePower = MIN_STRAFE_POWER * Math.signum(proportionalPower);
//                } else {
//                    strafePower = proportionalPower;
//                }
//
//                strafePower = Range.clip(strafePower, -MAX_STRAFE_POWER, MAX_STRAFE_POWER);
//                drivetrain.driveFieldCentric(0, strafePower, 0);
//                break;
//
//            case MOVING_ARM_AND_CLAW:
//                arm.setSliderLength(lockedSliderTarget);
//                endEffector.rotateToAutoIntakeAngle(lockedRotationTarget);
//                if (Math.abs(arm.getSliderLength() - lockedSliderTarget) < 2.5 || timer.seconds() > MOVEMENT_TIMEOUT_S) {
//                    currentState = State.LOWERING_CLAW;
//                }
//                break;
//            case LOWERING_CLAW:
//                endEffector.pivotToIntakePosition();
//                timer.reset();
//                currentState = State.GRIPPING;
//                break;
//            case GRIPPING:
//                if (timer.milliseconds() > 500) {
//                    endEffector.gripperGrip();
//                    timer.reset();
//                    currentState = State.LIFTING_CLAW;
//                }
//                break;
//            case LIFTING_CLAW:
//                if (timer.milliseconds() > 500) {
//                    endEffector.pivotToStagingPosition();
//                    timer.reset();
//                    currentState = State.STAGING;
//                }
//                break;
//            case STAGING:
//                endEffector.gripperGrip();
//                arm.setSliderLength(0);
//                if (Math.abs(arm.getSliderLength()) < 2.0 || timer.seconds() > 2.0) {
//                    new GoToSampleStagingCommand(arm, endEffector).schedule();
//                    currentState = State.FINISHED;
//                }
//                break;
//            case FAIL:
//                currentState = State.FINISHED;
//                break;
//            case FINISHED:
//                break;
//        }
//    }
//
//    private void lockOntoTargetAndDecideAction() {
//        lockedSliderTarget = Math.max(10.0, vision.getTargetSliderExtension());
//        lockedRotationTarget = vision.getTargetClawRotation();
//
//        double angleToTargetRad = Math.toRadians(Constants.Vision.LIMELIGHT_PITCH_ANGLE_DEG + vision.getTargetVerticalAngle());
//        double forwardDistance = (angleToTargetRad < 0) ? Constants.Vision.LIMELIGHT_HEIGHT_CM / Math.tan(-angleToTargetRad) : 30.0;
//
//        double desiredHorizontalOffsetCm = -Constants.Vision.LIMELIGHT_HORIZONTAL_OFFSET_CM;
//        lockedHorizontalAngleGoal = Math.toDegrees(Math.atan2(desiredHorizontalOffsetCm, forwardDistance));
//
//        if (Math.abs(vision.getTargetHorizontalAngle() - lockedHorizontalAngleGoal) > STRAFE_TOLERANCE_DEG) {
//            currentState = State.STRAFING_TO_ALIGN;
//        } else {
//            timer.reset();
//            currentState = State.MOVING_ARM_AND_CLAW;
//        }
//    }
//
//    @Override
//    public boolean isFinished() { return currentState == State.FINISHED; }
//
//    @Override
//    public void end(boolean interrupted) {
//        drivetrain.stop();
//        if (interrupted) {
//            arm.setSliderLength(arm.getSliderLength());
//        }
//    }
//}
