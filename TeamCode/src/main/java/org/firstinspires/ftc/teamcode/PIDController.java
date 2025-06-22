package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
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