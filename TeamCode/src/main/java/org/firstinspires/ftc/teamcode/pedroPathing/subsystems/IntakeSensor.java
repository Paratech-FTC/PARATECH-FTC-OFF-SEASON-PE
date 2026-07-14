package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSensor {

    private final RevColorSensorV3 sensor1;
    private final RevColorSensorV3 sensor2;

    public IntakeSensor(HardwareMap hardwareMap) {
        sensor1 = hardwareMap.get(RevColorSensorV3.class, "intakeSensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "intakeSensor2");
    }

    public double getDistance1() {
        return sensor1.getDistance(DistanceUnit.CM);
    }

    public double getDistance2() {
        return sensor2.getDistance(DistanceUnit.CM);
    }

    public boolean hasArtifact() {
        return getDistance1() < 4.0 || getDistance2() < 4.0;
    }

    public void periodic() {
    }
}