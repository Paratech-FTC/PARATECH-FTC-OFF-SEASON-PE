package org.firstinspires.ftc.teamcode.pedroPathing.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.IntakeSensor;

@Autonomous(name = "Autozinho Blue Near", group = "Main")
public class autozinhoBlueNear extends OpMode {

    // Pedro Pathing & Timers
    private Follower follower;
    private Timer actionTimer = new Timer();
    private Timer rgbTimer = new Timer(); // Timer para controlar a transição do LED
    private int pathState = 0;

    // Subsistemas e Sensores
    private DcMotor intake, indexer;
    private DcMotorEx leftShooter, rightShooter;
    private Servo angulator, rgbIndicator;
    private IntakeSensor intakeSensor;

    // Constantes do Shooter e Alvos
    private final double TARGET_VELOCITY = 1800;
    private final double VELOCITY_TOLERANCE = 130;

    // Definição das Poses (Posições do Campo)
    private final Pose startPose = new Pose(39, 30, Math.toRadians(50));
    private final Pose shootPose = new Pose(16, 4, Math.toRadians(52));
    private final Pose waitSecondIntake = new Pose(13, 35, Math.toRadians(90));
    private final Pose intakePose = new Pose(-2, 0, Math.toRadians(90));
    private final Pose intakeEnd = new Pose(-2, 36, Math.toRadians(90));
    private final Pose intakeEndBack = new Pose(-2, 32, Math.toRadians(90));
    private final Pose openGate = new Pose(2, 33 , Math.toRadians(90));
    private final Pose openGateBack = new Pose(2, 30, Math.toRadians(90));
    private final Pose colectGate = new Pose(-5, 37, Math.toRadians(50));
    private final Pose colectGateBack = new Pose(-3, 34, Math.toRadians(50)); // <--- Pequena ré para o colectGate
    private final Pose colectGate2 = new Pose(-5, 40, Math.toRadians(40));
    private final Pose gateBack = new Pose(-6, 26, Math.toRadians(90));

    // Caminhos (Paths)
    private Path
            goShoot1, goToIntake, goToColect,
            goToOpenGate, openGateBackPath, goIntakeForward, goIntakeEndBackPath, goShoot2,
            goShoot3, goBackGate, goToColect2, goToFileira2,
            goShoot4, goToColectGateBack; // <--- Novo caminho de ré do colectGate

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        initHardware();

        follower.setStartingPose(startPose);
        buildPaths();
        rgbTimer.resetTimer(); // Inicia o timer do LED

        telemetry.addLine("Robô Pronto para o Autônomo!");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        intakeSensor.periodic();
        updateRGB(); // Atualiza as cores do LED continuamente

        autonomousPathUpdate();

        telemetry.addData("Estado Atual", pathState);
        telemetry.addData("Tem peça?", intakeSensor.hasArtifact() ? "SIM" : "NÃO");
        telemetry.addData("RPM do shooter", leftShooter.getVelocity());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(goShoot1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    stopAll();
                    follower.followPath(goToIntake);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(goIntakeForward);
                    pathState = 4;
                }
                break;

            case 4:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goIntakeEndBackPath);
                    pathState = 45;
                }
                break;

            case 45:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShoot2);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 6;
                }
                break;

            case 6:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    stopAll();
                    follower.followPath(goToOpenGate);
                    pathState = 7;
                }
                break;

            case 7:
                leftShooter.setPower(-.7);
                rightShooter.setPower(-.7);
                intake.setPower(1.0);
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 72;
                }
                break;

            case 72:
                if (actionTimer.getElapsedTimeSeconds() >= 0.15) {
                    follower.followPath(openGateBackPath);
                    pathState = 75;
                }
                break;

            // --- ALTERAÇÃO APLICADA AQUI ---
            case 75:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    // Vai direto do openGateBack para o colectGate
                    follower.followPath(goToColect);
                    pathState = 76;
                }
                break;

            case 76:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    // Chegou no colectGate, reseta o timer para ficar 2 segundos parado lá com o intake ligado
                    stopAll();
                    actionTimer.resetTimer();
                    pathState = 77;
                }
                break;

            case 77:
                runIntakeWithSensor(); // Mantém o runIntakeWithSensor ligado durante os 2 segundos
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    // Passou 2 segundos, dá a pequena ré
                    follower.followPath(goToColectGateBack);
                    pathState = 78;
                }
                break;

            case 78:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    // Terminou a ré, volta para o colectGate
                    follower.followPath(goToColect);
                    pathState = 79;
                }
                break;

            case 79:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    // Chegou de volta no colectGate, segue para o goBackGate
                    follower.followPath(goBackGate);
                    pathState = 8;
                }
                break;
            // -------------------------------

            case 8:
                if (!follower.isBusy()) {
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    // Substituído os estados antigos de openGateOut por ir direto pro goShoot3 (ou mantido o fluxo restante)
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShoot3);
                    pathState = 12;
                }
                break;

            case 12:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 13;
                }
                break;

            case 13:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    stopAll();
                    pathState = 14;
                }
                break;

            case 14:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goToFileira2);
                    pathState = 15;
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShoot4);
                    pathState = 16;
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    stopAll();
                    actionTimer.resetTimer();
                    pathState = 17;
                }
                break;

            case 17:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    stopAll();
                    pathState = 18;
                }
                break;
        }
    }

    // --- MÉTODOS DOS SUBSISTEMAS ---
    private void runShooterSequence() {
        leftShooter.setVelocity(TARGET_VELOCITY);
        rightShooter.setVelocity(TARGET_VELOCITY);

        boolean isShooterReady = Math.abs(leftShooter.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE;

        if (isShooterReady) {
            intake.setPower(1);
            indexer.setPower(-1);
        } else {
            indexer.setPower(0);
        }
    }

    private void runIntakeWithSensor() {
        intake.setPower(1);
        leftShooter.setPower(-.6);
        rightShooter.setPower(-.6);
        indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.6);
    }

    private void stopAll() {
        intake.setPower(0);
        indexer.setPower(0);
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    private void updateRGB() {
        rgbIndicator.setPosition(.29);
    }

    private void buildPaths() {
        goShoot1 = new Path(new BezierLine(startPose, shootPose));
        goShoot1.setConstantHeadingInterpolation(shootPose.getHeading());

        goToIntake = new Path(new BezierLine(shootPose, intakePose));
        goToIntake.setConstantHeadingInterpolation(intakePose.getHeading());

        goIntakeForward = new Path(new BezierLine(intakePose, intakeEnd));
        goIntakeForward.setConstantHeadingInterpolation(intakeEnd.getHeading());
        goIntakeForward.setBrakingStrength(0.3);

        goIntakeEndBackPath = new Path(new BezierLine(intakeEnd, intakeEndBack));
        goIntakeEndBackPath.setConstantHeadingInterpolation(intakeEndBack.getHeading());

        goShoot2 = new Path(new BezierLine(intakeEndBack, shootPose));
        goShoot2.setConstantHeadingInterpolation(shootPose.getHeading());

        goShoot3 = new Path(new BezierLine(gateBack, shootPose));
        goShoot3.setConstantHeadingInterpolation(shootPose.getHeading());

        goShoot4 = new Path(new BezierLine(waitSecondIntake, shootPose));
        goShoot4.setConstantHeadingInterpolation(shootPose.getHeading());

        goToOpenGate = new Path(new BezierLine(shootPose, openGate));
        goToOpenGate.setConstantHeadingInterpolation(openGate.getHeading());

        openGateBackPath = new Path(new BezierLine(openGate, openGateBack));
        openGateBackPath.setConstantHeadingInterpolation(openGate.getHeading());

        goToFileira2 = new Path(new BezierLine(shootPose, waitSecondIntake));
        goToFileira2.setConstantHeadingInterpolation(waitSecondIntake.getHeading());
        goToFileira2.setBrakingStrength(0.3);

        // Caminho direto do openGateBack para o colectGate
        goToColect = new Path(new BezierLine(openGateBack, colectGate));
        goToColect.setConstantHeadingInterpolation(colectGate.getHeading());

        // Caminho de pequena ré saindo do colectGate
        goToColectGateBack = new Path(new BezierLine(colectGate, colectGateBack));
        goToColectGateBack.setConstantHeadingInterpolation(colectGate.getHeading());

        goToColect2 = new Path(new BezierLine(gateBack, colectGate2));
        goToColect2.setConstantHeadingInterpolation(colectGate2.getHeading());

        goBackGate = new Path(new BezierLine(colectGate, gateBack));
        goBackGate.setConstantHeadingInterpolation(colectGate.getHeading());
    }

    private void initHardware() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        angulator = hardwareMap.get(Servo.class, "angulator");
        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");
        intakeSensor = new IntakeSensor(hardwareMap);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angulator.setPosition(0.73);
    }
}