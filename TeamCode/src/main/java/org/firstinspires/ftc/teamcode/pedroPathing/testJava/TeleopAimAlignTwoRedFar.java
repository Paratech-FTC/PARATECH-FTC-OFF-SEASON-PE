package org.firstinspires.ftc.teamcode.pedroPathing.testJava;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSensor;

@TeleOp(name = "Red Far Teleop Aim Align Two Gamepads")
public class TeleopAimAlignTwoRedFar extends LinearOpMode {

    private Follower follower;
    private double headingOffset = 0;
    private boolean lastA = false;
    private final Pose leftGoal = new Pose(40,40);
    private final Pose rightGoal = new Pose(40,-40);
    private boolean aimAlign = false;

    private Pose currentTarget = leftGoal;
    private boolean autoAngulator = true;

    private double kP = 1.4;
    private double headingDeadzone = Math.toRadians(1.5);

    private DcMotor intake, indexer;
    private DcMotorEx leftShooter, rightShooter;
    private Servo angulator, rgbIndicator;

    // Sensores
    private IntakeSensor intakeSensor;
    private DistanceSensor distRight;

    // Configurações do servo
    private final double servoPosMin = 0.65;
    private final double servoPosMax = .45;
    private final double distStart = 100;
    private final double distEnd = 20;

    public final double targetVelocity = 2000;
    public final double velocityTolerance = 100;

    // Valores PWM do RGB (ajuste se necessário conforme manual da goBILDA)
    private final double COLOR_RED = .29;
    private final double COLOR_GREEN = .5;
    private final double COLOR_OFF = 0;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        // Corrigido para iniciar na pose correta com 52 graus em radianos
        follower.setStartingPose(new Pose(-35, -7, Math.toRadians(-25)));
        initSubsystems();

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        follower.startTeleopDrive();

        while(opModeIsActive()) {
            follower.update();
            Pose robotPose = follower.getPose();

            // Inputs
            if (gamepad1.x) { currentTarget = leftGoal; aimAlign = true; }
            if (gamepad1.b) { currentTarget = rightGoal; aimAlign = true; }

            // Lógica do Angulador
            if (autoAngulator) {
                double distance = Math.hypot(currentTarget.getX() - robotPose.getX(), currentTarget.getY() - robotPose.getY());
                double t = Range.clip((distance - distStart) / (distEnd - distStart), 0, 1);
                double targetPos = servoPosMin + (t * (servoPosMax - servoPosMin));
                angulator.setPosition(targetPos);
            }

            // Drive control
            if (gamepad1.a && !lastA) headingOffset = robotPose.getHeading();
            lastA = gamepad1.a;

            double fieldHeading = robotPose.getHeading();
            if (Math.abs(gamepad1.right_stick_x) > 0.1) aimAlign = false;

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn;

            if (aimAlign) {
                double targetHeading = Math.atan2(currentTarget.getY() - robotPose.getY(), currentTarget.getX() - robotPose.getX());
                double error = Math.atan2(Math.sin(targetHeading - fieldHeading), Math.cos(targetHeading - fieldHeading));
                turn = (Math.abs(error) < headingDeadzone) ? 0 : limit(error * kP);
            } else {
                turn = -gamepad1.right_stick_x;
            }

            // Field-centric desativado (false no último parâmetro) para testar o alinhamento puro
            follower.setTeleOpDrive(forward, strafe, turn, false);

            subsystems();
            updateRGB();

            double currentVelocity = leftShooter.getVelocity();
            boolean shooterReady = Math.abs(currentVelocity - targetVelocity) < velocityTolerance;

            telemetry.addData("Aim Align Active", aimAlign);
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("X", robotPose.getX());
            telemetry.addData("Y", robotPose.getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(robotPose.getHeading()));
            telemetry.update();
        }
    }

    private void initSubsystems() {
        intake = hardwareMap.get(DcMotor.class,"intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        angulator = hardwareMap.get(Servo.class, "angulator");
        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");

        intakeSensor = new IntakeSensor(hardwareMap);
        distRight = hardwareMap.get(DistanceSensor.class, "distSensor");

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angulator.scaleRange(0.2, 1);
    }

    private void subsystems() {
        intakeSensor.periodic();
        if (gamepad2.y) {
            leftShooter.setVelocity(targetVelocity);
            rightShooter.setVelocity(targetVelocity);
            if (Math.abs(leftShooter.getVelocity() - targetVelocity) < velocityTolerance) {
                indexer.setPower(-1);
                intake.setPower(0.8);
            } else {
                indexer.setPower(0);
            }
        } else {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);
            if (gamepad2.right_bumper) {
                intake.setPower(1.0);
                indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.7);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1.0);
                indexer.setPower(1.0);
            } else {
                intake.setPower(0);
                indexer.setPower(0);
            }
        }
    }

    private void updateRGB() {
        boolean hasArtifact = intakeSensor.hasArtifact();
        boolean distDetected = (distRight.getDistance(DistanceUnit.CM) < 10);

        if (hasArtifact && distDetected) {
            rgbIndicator.setPosition(COLOR_GREEN);
        } else if (hasArtifact) {
            rgbIndicator.setPosition(COLOR_RED);
        } else {
            rgbIndicator.setPosition(COLOR_OFF);
        }
    }

    private double limit(double value) {
        return Math.max(-1, Math.min(1, value));
    }
}