package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotor intake;
    private final DcMotor indexer;

    private final IntakeSensor intakeSensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");

        intakeSensor = new IntakeSensor(hardwareMap);

    }

    public void initialize() {

        stop();

    }

    public void periodic() {

        intakeSensor.periodic();

    }

    public void intake() {

        if (intakeSensor.hasArtifact()) {

            stop();

        } else {

            intake.setPower(1.0);
            indexer.setPower(-0.4);

        }

    }

    public void reverse() {

        intake.setPower(-1.0);
        indexer.setPower(0.4);

    }

    public void stop() {

        intake.setPower(0);
        indexer.setPower(0);

    }

    public boolean hasArtifact() {

        return intakeSensor.hasArtifact();

    }

    public double getDistance1() {

        return intakeSensor.getDistance1();

    }

    public double getDistance2() {

        return intakeSensor.getDistance2();

    }

}