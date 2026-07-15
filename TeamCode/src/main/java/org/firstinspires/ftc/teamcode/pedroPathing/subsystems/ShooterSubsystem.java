package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterSubsystem {
    private DcMotorEx leftShooter, rightShooter;
    public final double targetVelocity = 6000;
    public final double velocityTolerance = 80;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShooterPower(double velocity) {
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(velocity);
    }

    public boolean isReady() {
        return Math.abs(leftShooter.getVelocity() - targetVelocity) < velocityTolerance;
    }

    public double getVelocity() {
        return leftShooter.getVelocity();
    }
}