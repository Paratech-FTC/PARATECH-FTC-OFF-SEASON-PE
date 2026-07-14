package org.firstinspires.ftc.teamcode.pedroPathing.testJava;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSensor;


@TeleOp(name = "Teleop Aim Align")
public class TeleopAimAlign extends LinearOpMode {


    private Follower follower;

    private double headingOffset = 0;
    private boolean lastA = false;
    private final Pose leftGoal = new Pose(40,40);
    private final Pose rightGoal = new Pose(40,-40);
    private boolean aimAlign = false;
    private Pose currentTarget = null;

    private double kP = 1.4;

    private double headingDeadzone = Math.toRadians(1.5);

    private DcMotor intake, rightShooter, leftShooter, indexer;

    private Servo angulator;

    private final double servoHigh = 0.35;
    private final double servoLow = 1.00;

    private final double minDistance = 0;
    private final double maxDistance = 95;

    private IntakeSensor intakeSensor;



    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        initSubsystems();
        angulator.setPosition(.45);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        if(isStopRequested())
            return;

        follower.startTeleopDrive();

        while(opModeIsActive()) {
            follower.update();

            Pose robotPose = follower.getPose();

            if(currentTarget != null){

                double distance = robotPose.distanceFrom(currentTarget);

                double t = (distance - minDistance) / (maxDistance - minDistance);
                t = Math.max(0, Math.min(1, t));

                double servoPosition =
                        servoHigh + t * (servoLow - servoHigh);

                angulator.setPosition(servoPosition);

                telemetry.addData("Distance", distance);
                telemetry.addData("Servo", servoPosition);
            }

            if (gamepad1.a && !lastA) {
                headingOffset = robotPose.getHeading();
            }

            lastA = gamepad1.a;

            double fieldHeading = robotPose.getHeading();

            double driverHeading = fieldHeading - headingOffset;
            driverHeading = Math.atan2(
                    Math.sin(driverHeading),
                    Math.cos(driverHeading)
            );

            if (gamepad1.x) {
                currentTarget = leftGoal;
                aimAlign = true;
            }

            if (gamepad1.b) {
                currentTarget = rightGoal;
                aimAlign = true;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                aimAlign = false;
            }

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            double turn;

            if (aimAlign && currentTarget != null) {

                double targetHeading = Math.atan2(
                        currentTarget.getY() - robotPose.getY(),
                        currentTarget.getX() - robotPose.getX()
                );

                double error = Math.atan2(
                        Math.sin(targetHeading - fieldHeading),
                        Math.cos(targetHeading - fieldHeading)
                );

                if (Math.abs(error) < headingDeadzone) {
                    turn = 0;
                } else {
                    turn = limit(error * kP);
                }

                telemetry.addData("Goal", currentTarget == leftGoal ? "LEFT" : "RIGHT");
                telemetry.addData("Error", Math.toDegrees(error));

            } else {
                turn = -gamepad1.right_stick_x;
            }

            double tempForward =
                    forward * Math.cos(driverHeading) -
                            strafe * Math.sin(driverHeading);

            double tempStrafe =
                    forward * Math.sin(driverHeading) +
                            strafe * Math.cos(driverHeading);

            forward = tempForward;
            strafe = tempStrafe;

            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    turn,
                    false
            );

            subsystems();

            telemetry.addData("X", robotPose.getX());
            telemetry.addData("Y", robotPose.getY());
//            telemetry.addData("Heading", Math.toDegrees(robotHeading));
            telemetry.addData("Aim", aimAlign);
            telemetry.update();
        }
    }

    private void initSubsystems() {
        intake = hardwareMap.get(DcMotor.class,"intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        intakeSensor = new IntakeSensor(hardwareMap);

        angulator = hardwareMap.get(Servo.class, "angulator");
        angulator.scaleRange(0, 1.0);
    }
    private void subsystems() {
        intakeSensor.periodic();

        if (gamepad1.y){
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
            angulator.setPosition(.5);
        }

        if(gamepad1.dpad_up){
            angulator.setPosition(0);
        }

        if (gamepad1.right_bumper) {
            intake.setPower(1.0);

            if (intakeSensor.hasArtifact() && !gamepad1.y) {
                indexer.setPower(0);
            } else if (!intakeSensor.hasArtifact() && gamepad1.y){
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
    }

    private double limit(double value) {
        if(value > 1)
            return 1;
        if(value < -1)
            return -1;

        return value;
    }
}