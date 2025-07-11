package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Project1Hardware {
    DcMotorEx sliderLeft, sliderRight, arm;
    ServoImplEx differentialLeft, differentialRight, clawIntake, clawScoring;
    IMU imu;
    Drivetrain drivetrain;
    DifferentialModule differential;
    UltrasonicModule ultraLeft, ultraRight, ultraBack;
    Mode mode = Mode.SAMPLE;

    final double INITIAL_ANGLE = -35.7;
    final double CPR = ((((1 + ((double) 46 / 11))) * (1 + ((double) 46 / 11))) * 28);  // ~751.8
    final double PPR = ((1 + ((double) 46 / 11)) * 28);  // ~118.1
    boolean intakeUp = false, clawIntakeOpen, clawScoringOpen;

    public static double P = 22;
    public static double I = 5;
    public static double D = 5;
    public static double F = 5;

    private void init(@NonNull HardwareMap hardwareMap) {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        sliderLeft = hardwareMap.get(DcMotorEx.class, "sliderLeft");
        sliderRight = hardwareMap.get(DcMotorEx.class, "sliderRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        differentialLeft = hardwareMap.get(ServoImplEx.class, "differentialLeft");
        differentialRight = hardwareMap.get(ServoImplEx.class, "differentialRight");
        clawIntake = hardwareMap.get(ServoImplEx.class, "clawIntake");
        clawScoring = hardwareMap.get(ServoImplEx.class, "clawScoring");
        ultraLeft = new UltrasonicModule(hardwareMap, "triggerLeft", "echoLeft");
        ultraRight = new UltrasonicModule(hardwareMap, "triggerRight", "echoRight");
        ultraBack = new UltrasonicModule(hardwareMap, "triggerBack", "echoBack");
        imu = hardwareMap.get(IMU.class, "imu");

        // Call methods that must be called regardless of Autonomous or TeleOp.
        // Put OpMode-specific method calls in their respective initialisation methods.
        // Set directions of actuators and modes of drivetrain motors.
        // Do not set modes of sliders or reset encoders here.

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F);
        sliderLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        sliderRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        differentialLeft.setDirection(Servo.Direction.REVERSE);
        differentialRight.setDirection(Servo.Direction.FORWARD);
        clawIntake.setDirection(Servo.Direction.FORWARD);
        clawScoring.setDirection(Servo.Direction.FORWARD);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
        differential = new DifferentialModule(differentialLeft, differentialRight, 0, 0.2);
    }

    public void resetEncoders() {
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void applyAutonomousData(@Nullable Project1Hardware autonomous) {
        if (autonomous != null) {
            intakeUp = autonomous.intakeUp;
            clawIntakeOpen = autonomous.clawIntakeOpen;
            clawScoringOpen = autonomous.clawScoringOpen;
            differential = autonomous.differential;
            mode = autonomous.mode;
        }
    }

    public void copyToStorage(Pose2d lastPose, Operation operation) {
        Storage.robot = this;
        Storage.lastPose = lastPose;
        Storage.lastOpMode = operation;
    }

    // OpMode-specific initialisation methods go here. Note that there is no default constructor
    // for this class, so <instance>.init(HardwareMap hardwareMap) must be called.

    /**
     * Factory method for the hardware class, and calls methods required for autonomous
     * initialisation.
     * @param hardwareMap HardwareMap object, to be passed from the OpMode class.
     * @return a {@link Project1Hardware} robot object.
     */
    @NonNull public static Project1Hardware initAutonomous(HardwareMap hardwareMap) {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap);
        robot.resetEncoders();
        robot.imu.resetYaw();
        return robot;
    }

    /**
     * Factory method for the hardware class, and calls methods required for teleoperated
     * initialisation.
     * @param hardwareMap HardwareMap object, to be passed from the OpMode class.
     * @return a {@link Project1Hardware} robot object.
     */
    @NonNull public static Project1Hardware initTeleoperated(HardwareMap hardwareMap) {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap);
        robot.applyAutonomousData(Storage.robot);
        return robot;
    }

    @NonNull public static Project1Hardware initUndefined(HardwareMap hardwareMap) {
        Project1Hardware robot = new Project1Hardware();
        robot.init(hardwareMap);
        return robot;
    }

    // Hardware methods go below.

    public void setSlider(int k, double power) {
        sliderLeft.setTargetPosition(k);
        sliderRight.setTargetPosition(k);
        sliderLeft.setPower(power);
        sliderRight.setPower(power);
        sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSliderPower(double power) {
        sliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderLeft.setPower(power);
        sliderRight.setPower(power);
    }

    public int getSlider() {return sliderLeft.getCurrentPosition();}

    public boolean sliderInPosition(int tolerance) {
        return Math.abs(getSlider() - sliderLeft.getTargetPosition()) <= tolerance;
    }

    public void setSlider(int k) {setSlider(k, 1);}
    public boolean sliderInPosition() {return sliderInPosition(20);}

    public double getSliderInches() {return getSlider() / PPR * 112 / 25.4;}
    private double encoderToAngle(double encoder) {return encoder / CPR * 360 + INITIAL_ANGLE;}
    public int inchesToEncoder(double inches) {return (int) Math.round(inches * 25.4 / 112 * PPR);}

    public double getArmAngle() {
        int position = arm.getCurrentPosition();
        double revolutions = position / CPR;
        return revolutions * 360 + INITIAL_ANGLE;
    }

    public void setArmAngle(double angle, double power) {
        arm.setTargetPosition((int) Math.round((angle - INITIAL_ANGLE) / 360 * CPR));
        arm.setPower(power);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean armInPosition(double toleranceAngle) {
        return Math.abs(getArmAngle() - encoderToAngle(arm.getTargetPosition())) <= toleranceAngle;
    }

    public void powerOffArm() {arm.setPower(0);}
    public void setArmAngle(double angle) {setArmAngle(angle, 1);}
    public void setArmTransfer() {setArmAngle(INITIAL_ANGLE, 0.3);}
    public void setArmBasketFront() {setArmAngle(45);}
    public void setArmBasketRear() {setArmAngle(135);}
    public void setArmScoring() {setArmAngle(15);}
    public void setArmChambered() {setArmAngle(168);}
    public boolean armInPosition() {return armInPosition(2.5);}

    public void clawIntakeOpen() {clawIntake.setPosition(0.32); clawIntakeOpen = true;}
    public void clawIntakeClose() {clawIntake.setPosition(0); clawIntakeOpen = false;}
    public void clawScoringOpen() {clawScoring.setPosition(0.78); clawScoringOpen = true;}
    public void clawScoringOpenMax() {clawScoring.setPosition(0.9); clawScoringOpen = true;}
    public void clawScoringClose() {clawScoring.setPosition(0.62); clawScoringOpen = false;}

    public void intakeUp2() {
        differentialLeft.setPosition(0);
        differentialRight.setPosition(0);
        intakeUp = true;
    }

    public void intakeUp() {
        differentialLeft.setPosition(1);
        differentialRight.setPosition(1);
        intakeUp = true;
    }

    public void intakeDown() {differential.setPosition(0.28, 0); intakeUp = false;}
    public void intakeSetOrientation(double angle) {differential.setOrientation(angle);}

    public double getIMU() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
    public double getIMUDegrees() {return Math.toDegrees(getIMU());}

    // Utility and robot classes go below.

    public static class Drivetrain {
        DcMotor frontLeft, frontRight, backLeft, backRight;

        public Drivetrain(DcMotor fL, DcMotor fR, DcMotor bL, DcMotor bR) {
            this.frontLeft = fL;
            this.frontRight = fR;
            this.backLeft = bL;
            this.backRight = bR;
        }

        public void remote(double vertical, double horizontal, double pivot, double heading) {
            double theta = 2 * Math.PI + Math.atan2(vertical, horizontal) - heading;
            double power = Math.hypot(horizontal, vertical);

            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double max = Math.max(Math.abs(sin), Math.abs(cos));

            double powerFL = power * (cos / max) + pivot;
            double powerFR = power * (sin / max) - pivot;
            double powerBL = power * -(sin / max) - pivot;
            double powerBR = power * -(cos / max) + pivot;

            frontLeft.setPower(-powerFL);
            frontRight.setPower(-powerFR);
            backLeft.setPower(powerBL);
            backRight.setPower(powerBR);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public static class DifferentialModule {
        ServoImplEx left, right;
        public final static double HALF = 0.35;
        private double offsetLeft = 0.0;
        private double offsetRight = 0.0;
        private double base, diffLeft, diffRight;

        public DifferentialModule(ServoImplEx left, ServoImplEx right) {
            this.left = left;
            this.right = right;
            diffLeft = 0;
            diffRight = 0;
        }

        public DifferentialModule(ServoImplEx left, ServoImplEx right, double offsetL, double offsetR) {
            this.left = left;
            this.right = right;
            diffLeft = 0;
            diffRight = 0;
            offsetLeft = offsetL;
            offsetRight = offsetR;
        }

        /** Sets servos' positions. Call after a method. */
        private void apply() {
            left.setPosition(base + offsetLeft + diffLeft);
            right.setPosition(base + offsetRight + diffRight);
        }

        /**
         * Sets differences of the left servo and the right servo from the base (pitch).
         * @param left Difference between left servo and base. Pass in <code>2</code> for no change.
         * @param right Difference between right servo and base. Pass in <code>2</code> for no
         *              change.
         */
        private void setDifferences(double left, double right) {
            if (left <= 1) this.diffLeft = left;
            if (right <= 1) this.diffRight = right;
            apply();
        }

        /**
         * Sets differences of the left servo and the right servo from the base (pitch).
         * Shorthand for {@link #setDifferences(double, double)} when both values are equal in
         * magnitude but opposite in direction.
         * @param value Difference between servo and base.
         */
        public final void setDifferences(double value) {setDifferences(value, -value);}

        /** Enables power on both arm servos. */
        public final void setPwmEnable() {
            right.setPwmEnable();
            left.setPwmEnable();
        }

        /** Disables power on both arm servos. */
        public final void setPwmDisable() {
            left.setPwmDisable();
            right.setPwmDisable();
        }

        /**
         * Gets the current base pitch. Note that this does not reflect the orientation of the
         * module or the actual position values of the servos.
         * @return Pitch of the scoring module.
         */
        public final double getPitch() {return this.base;}

        /**
         * Sets the pitch of the scoring set. This does not interfere with the orientation (roll)
         * of the scoring module.
         * @param target The target value, relative to the servos.
         */
        public final void setPitch(double target) {
            this.base = target;
            apply();
        }

        /**
         * Sets the pitch and the differences of the servos. A combination of
         * {@link #setDifferences(double, double) setDifferences()} and
         * {@link #setPitch(double) setPitch()}.
         * @param pitch The target pitch, relative to the servos.
         * @param diffLeft Difference between left servo and base. Pass in <code>2</code> for no
         *                 change.
         * @param diffRight Difference between right servo and base. Pass in <code>2</code> for no
         *                  change.
         */
        private void setValues(double pitch, double diffLeft, double diffRight) {
            if (diffLeft <= 1) this.diffLeft = diffLeft;
            if (diffRight <= 1) this.diffRight = diffRight;
            this.base = pitch;
            apply();
        }

        /** Sets the orientation of scoring to a specific angle.
         * @param target The target (in degrees) for the module to rotate to.
         */
        public final void setOrientation(double target) {
            double distance = HALF * target / 90;
            setDifferences(distance, -distance);
        }

        public double getOrientation() {return diffLeft / HALF * 90;}

        public final void setPosition(double pitch, double orientation) {
            double difference = HALF * orientation / 90;
            setValues(pitch, difference, -difference);
        }
    }

    public static class UltrasonicModule {
        DigitalChannel trigger;
        DigitalChannel echo;

        public UltrasonicModule(@NonNull HardwareMap hardwareMap, String t, String e) {
            trigger = hardwareMap.get(DigitalChannel.class, t);
            echo = hardwareMap.get(DigitalChannel.class, e);
            trigger.setMode(DigitalChannel.Mode.OUTPUT);
            echo.setMode(DigitalChannel.Mode.INPUT);
        }

        public synchronized double getDistance() throws InterruptedException {
            // 1. Send trigger pulse
            trigger.setState(false);
            Thread.sleep(2); // 2 ms
            trigger.setState(true);
            Thread.sleep(0, 10000); // 10 µs
            trigger.setState(false);

            // 2. Wait for echo to go HIGH
            ElapsedTime timer = new ElapsedTime();
            while (!echo.getState()) {
                if (timer.milliseconds() > 100) return -1; // timeout
            }
            double startTime = timer.nanoseconds();

            // 3. Wait for echo to go LOW
            while (echo.getState()) {
                if (timer.milliseconds() > 100) return -1; // timeout
            }
            double endTime = timer.nanoseconds();

            // 4. Calculate duration and distance
            double durationMicro = (endTime - startTime) / 1000.0; // ns to µs
            return durationMicro / 58.2 / 2.54; // Distance in cm
        }
    }

    public static class PIDController {
        double kP, kI, kD, setpoint, lastError;
        private double integral;
        private double lastTime;
        ElapsedTime t;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP; this.kI = kI; this.kD = kD;
            setpoint = 0;
            lastError = 0;
            integral = 0;
            t = new ElapsedTime();
        }

        public void reset() {
            lastError = 0;
            integral = 0;
            t.reset();
        }

        public double calculate(double setpoint, double input, double dt) {
            final double error = setpoint - input;
            this.integral += error * dt;
            final double derivative = (error - lastError) / dt;
            final double output = kP * error + kI * integral + kD * derivative;
            lastError = error;

            return output;
        }
    }

    public enum Mode {SAMPLE, SPECIMEN}
    public enum Operation {AUTONOMOUS, TELEOPERATED}
}
