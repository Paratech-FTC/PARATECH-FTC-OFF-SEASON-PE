package org.firstinspires.ftc.teamcode.pedroPathing.testJava;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    // Agora o target começa valendo algo (leftGoal) para não dar erro
    private Pose currentTarget = leftGoal;

    private boolean autoAngulator = false;

    private double kP = 1.4;
    private double headingDeadzone = Math.toRadians(1.5);

    private DcMotor intake, indexer;
    private DcMotorEx leftShooter, rightShooter;
    private Servo angulator;

    private final double servoHigh = 0.35;
    private final double servoLow = 1.00;
    private final double minDistance = 40;
    private final double maxDistance = 0;
    public final double targetVelocity = 5700;
    public final double velocityTolerance = 100;

    private IntakeSensor intakeSensor;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        initSubsystems();
        angulator.setPosition(.3);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        follower.startTeleopDrive();

        while(opModeIsActive()) {
            follower.update();
            Pose robotPose = follower.getPose();

            // Lógica de seleção de alvo
            if (gamepad1.x) { currentTarget = leftGoal; aimAlign = true; }
            if (gamepad1.b) { currentTarget = rightGoal; aimAlign = true; }

            // Lógica do Angulator Automático
            if (gamepad1.dpad_right) autoAngulator = true;
            if (gamepad1.dpad_up) {
                autoAngulator = false;
                angulator.setPosition(0.1);
            }

            // Usando a pose atual do robô e as coordenadas do alvo atual
            if (autoAngulator) {
                double distance = Math.hypot(currentTarget.getX() - robotPose.getX(), currentTarget.getY() - robotPose.getY());
                double t = Math.max(0, Math.min(1, (distance - minDistance) / (maxDistance - minDistance)));
                angulator.setPosition(servoHigh + t * (servoLow - servoHigh));

                telemetry.addData("Auto-Angulator", "ON");
                telemetry.addData("Target", currentTarget == leftGoal ? "LEFT" : "RIGHT");
                telemetry.addData("Distance", distance);
            } else {
                telemetry.addData("Auto-Angulator", "OFF");
            }

            // Controles de Drive
            if (gamepad1.a && !lastA) headingOffset = robotPose.getHeading();
            lastA = gamepad1.a;

            double fieldHeading = robotPose.getHeading();
            double driverHeading = Math.atan2(Math.sin(fieldHeading - headingOffset), Math.cos(fieldHeading - headingOffset));

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

            //follower.setTeleOpDrive(
                   // forward, // forward direto do joystick
                  //  strafe,  // strafe direto do joystick
                //    turn,
              //      true     // TRUE ativa o Field Oriented
            //);

            follower.setTeleOpDrive(
                    forward * Math.cos(driverHeading) - strafe * Math.sin(driverHeading),
                    forward * Math.sin(driverHeading) + strafe * Math.cos(driverHeading),
                    turn, false
            );

            subsystems();

            double currentVel = leftShooter.getVelocity();
            telemetry.addData("Shooter Vel Real", currentVel);
            telemetry.addData("Shooter Ready", Math.abs(currentVel - targetVelocity) < velocityTolerance);
            telemetry.addData("Shooter Ready", Math.abs(leftShooter.getVelocity() - targetVelocity) < velocityTolerance);
            telemetry.update();
        }
    }

    private void initSubsystems() {
        intake = hardwareMap.get(DcMotor.class,"intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        intakeSensor = new IntakeSensor(hardwareMap);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angulator = hardwareMap.get(Servo.class, "angulator");
        angulator.scaleRange(0, 1.0);
    }

    private void subsystems() {
        intakeSensor.periodic();

        // 1. Lógica de Disparo (Prioridade)
        if (gamepad1.y) {
            leftShooter.setVelocity(targetVelocity);
            rightShooter.setVelocity(targetVelocity);

            boolean isShooterReady = Math.abs(leftShooter.getVelocity() - targetVelocity) < velocityTolerance;

            if (isShooterReady) {
                intake.setPower(1);
                indexer.setPower(-0.7);
            } else {
                // Shooter ligado, mas não pronto: para o indexer, mas mantém o intake
                // OU define ambos como 0 se preferir parar tudo
                indexer.setPower(0);
            }
        }
        // 2. Lógica de Intake Manual (Só acontece se NÃO estiver apertando Y)
        else {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);

            if (gamepad1.right_bumper) {
                intake.setPower(1.0);
                // Se tiver peça, para o indexer, se não, puxa
                indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.7);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1.0);
                indexer.setPower(1.0);
            } else {
                intake.setPower(0);
                indexer.setPower(0);
            }
        }
    }

    private double limit(double value) {
        return Math.max(-1, Math.min(1, value));
    }
}