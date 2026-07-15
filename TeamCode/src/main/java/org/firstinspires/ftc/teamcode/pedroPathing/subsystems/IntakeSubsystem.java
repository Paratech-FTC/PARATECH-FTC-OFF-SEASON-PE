package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private DcMotor intake, indexer;
    private IntakeSensor sensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        sensor = new IntakeSensor(hardwareMap);
    }

    public void update() { sensor.periodic(); }

    public void runIntake(double power) { intake.setPower(power); }
    public void runIndexer(double power) { indexer.setPower(power); }
    public boolean hasPiece() { return sensor.hasArtifact(); }
}