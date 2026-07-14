package org.firstinspires.ftc.teamcode.pedroPathing.testJava;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSensor;

@TeleOp(name = "xTeleop")
public class TeleopTeste extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor indexer = hardwareMap.get(DcMotor.class, "indexer");
        DcMotor leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        DcMotor rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        Servo angulator = hardwareMap.get(Servo.class,"angulator");

        angulator.scaleRange(0.2, 1);

        IntakeSensor intakeSensor = new IntakeSensor(hardwareMap);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Robot OK");
        telemetry.update();

        angulator.setPosition(.45);

        waitForStart();

        while (opModeIsActive()) {

            intakeSensor.periodic();

            if (gamepad1.a) {
                imu.resetYaw();
            }

            double strafeP = -gamepad1.left_stick_x;
            double driveP = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            double orientacaoRad =
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double drive =
                    driveP * Math.cos(orientacaoRad) -
                            strafeP * Math.sin(orientacaoRad);

            double strafe =
                    driveP * Math.sin(orientacaoRad) +
                            strafeP * Math.cos(orientacaoRad);

            double denominator =
                    Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rx), 1);

            double frontLeftPower = (drive + strafe + rx) / denominator;
            double backLeftPower = (drive - strafe + rx) / denominator;
            double frontRightPower = (drive - strafe - rx) / denominator;
            double backRightPower = (drive + strafe - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            if (gamepad1.x){
                leftShooter.setPower(-1);
                rightShooter.setPower(1);
                if(gamepad1.right_bumper){
                    intake.setPower(1);
                    indexer.setPower(-1);
                }
            } else {
                intake.setPower(0);
                indexer.setPower(0);
                leftShooter.setPower(0);
                rightShooter.setPower(0);
            }

            if(gamepad1.dpad_down){
                angulator.setPosition(1);
            }

            if(gamepad1.dpad_right){
                angulator.setPosition(.45);
            }

            if(gamepad1.dpad_up){
                angulator.setPosition(.35);
            }

            if (gamepad1.right_bumper) {
                intake.setPower(1.0);

                if (intakeSensor.hasArtifact() && !gamepad1.x) {
                    indexer.setPower(0);
                } else if (!intakeSensor.hasArtifact() && gamepad1.x){
                    indexer.setPower(-1);
                } else {
                    indexer.setPower(-.7);
                }
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
                indexer.setPower(1);
            } else {
                intake.setPower(0);
                indexer.setPower(0);
            }

            telemetry.addData("Heading", Math.toDegrees(orientacaoRad));
            telemetry.addData("Sensor 1", intakeSensor.getDistance1());
            telemetry.addData("Sensor 2", intakeSensor.getDistance2());
            telemetry.addData("Tem artefato?", intakeSensor.hasArtifact());
            telemetry.addData("Servo Positioning:", angulator.getPosition());
            telemetry.update();
        }

        intake.setPower(0);
        indexer.setPower(0);
    }
}