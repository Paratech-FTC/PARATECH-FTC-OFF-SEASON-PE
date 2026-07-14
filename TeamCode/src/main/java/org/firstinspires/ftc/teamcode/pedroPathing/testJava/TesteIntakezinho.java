package org.firstinspires.ftc.teamcode.pedroPathing.testJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSensor;

@TeleOp(name = "TesteIntakezinho")
public class TesteIntakezinho extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexer = hardwareMap.get(DcMotor.class, "indexer");

        IntakeSensor intakeSensor = new IntakeSensor(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            intakeSensor.periodic();

            if (intakeSensor.hasArtifact()) {
                indexer.setPower(0);
            } else {
                intake.setPower(1.0);
                indexer.setPower(-.7);
            }

            telemetry.addData("Distancia: ", intakeSensor.getDistance1());
            telemetry.addData("Tem artefato? ", intakeSensor.hasArtifact());
            telemetry.update();
        }
        intake.setPower(0);
    }
}