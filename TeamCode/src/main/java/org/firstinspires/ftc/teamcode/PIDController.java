package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private DcMotorEx motor;
    private double kP, kI, kD;
    private double setpoint = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(DcMotorEx motor, double kP, double kI, double kD) {
        this.motor = motor;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
        // Reset encoder and disable built-in PID
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // manula control only
    }

    public void setTargetPosition(double target) {
        this.setpoint = target;
        this.integralSum = 0;  // Reset integral term when updating target
        this.lastError = 0;
    }

    public void update() {
        double currentPos = motor.getCurrentPosition();
        double error = setpoint - currentPos;

        double deltaTime = timer.seconds();
        timer.reset();

        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // Prevent integral windup by clamping the integral sum
        integralSum = Math.max(-1000, Math.min(integralSum, 1000));

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        motor.setPower(output);
    }
}
