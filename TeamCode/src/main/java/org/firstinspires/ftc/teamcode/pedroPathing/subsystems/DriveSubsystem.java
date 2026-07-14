package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private final IMU imu;

    public DriveSubsystem(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(orientation));

        configureMotors();
    }

    private void configureMotors() {

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }

    public void robotCentric(double drive, double strafe, double turn) {

        double denominator = Math.max(
                Math.abs(drive) + Math.abs(strafe) + Math.abs(turn),
                1
        );

        setMotorPowers(
                (drive + strafe + turn) / denominator,
                (drive - strafe - turn) / denominator,
                (drive - strafe + turn) / denominator,
                (drive + strafe - turn) / denominator
        );

    }

    public void fieldCentric(double drive, double strafe, double turn) {

        driveWithHeading(
                drive,
                strafe,
                turn,
                getImuHeadingRadians()
        );

    }

    public void fieldCentricLimelight(double drive, double strafe, double turn) {

        double heading = 0;

        /*
         * colocar aq o heading da limelight
         */

        driveWithHeading(
                drive,
                strafe,
                turn,
                heading
        );

    }

    private void driveWithHeading(
            double drive,
            double strafe,
            double turn,
            double heading
    ) {

        double rotatedDrive =
                drive * Math.cos(heading) -
                        strafe * Math.sin(heading);

        double rotatedStrafe =
                drive * Math.sin(heading) +
                        strafe * Math.cos(heading);

        double denominator = Math.max(
                Math.abs(rotatedDrive) +
                        Math.abs(rotatedStrafe) +
                        Math.abs(turn),
                1
        );

        setMotorPowers(
                (rotatedDrive + rotatedStrafe + turn) / denominator,
                (rotatedDrive - rotatedStrafe - turn) / denominator,
                (rotatedDrive - rotatedStrafe + turn) / denominator,
                (rotatedDrive + rotatedStrafe - turn) / denominator
        );

    }

    private void setMotorPowers(
            double frontLeftPower,
            double frontRightPower,
            double backLeftPower,
            double backRightPower
    ) {

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }

    private double getImuHeadingRadians() {

        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

    }

    public double getHeadingDegrees() {

        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);

    }

    public void resetHeading() {

        imu.resetYaw();

    }

    public void initialize() {

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // limelight.start();
        // aqui vai ter outras inicializaçoes tb

    }

    public void stop() {

        setMotorPowers(
                0,
                0,
                0,
                0
        );

    }

}