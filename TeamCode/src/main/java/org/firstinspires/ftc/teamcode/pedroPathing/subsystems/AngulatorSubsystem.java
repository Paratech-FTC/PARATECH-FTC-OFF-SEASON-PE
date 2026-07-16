package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AngulatorSubsystem {
    private Servo angulator;
    private final double servoHigh = 0.35;
    private final double servoLow = 1.00;
    private final double minDistance = 0;
    private final double maxDistance = 95;

    public AngulatorSubsystem(HardwareMap hardwareMap) {
        angulator = hardwareMap.get(Servo.class, "angulator");
        angulator.scaleRange(0.2, 1.0);
        angulator.setPosition(0.1);
    }

    public void setPosition(double pos) {
        angulator.setPosition(pos);
    }

    public void updateByDistance(double distance) {
        double t = (distance - minDistance) / (maxDistance - minDistance);
        t = Math.max(0, Math.min(1, t));
        double servoPosition = servoHigh + t * (servoLow - servoHigh);
        angulator.setPosition(servoPosition);
    }
}