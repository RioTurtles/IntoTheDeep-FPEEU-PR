package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ActuatorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Project1Hardware robot = Project1Hardware.initAutonomous(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Slider", robot.getSlider());
            telemetry.addData("Arm(E)", robot.arm.getCurrentPosition());
            telemetry.addData("Arm(A)", robot.getArmAngle());
            telemetry.update();
        }
    }
}
