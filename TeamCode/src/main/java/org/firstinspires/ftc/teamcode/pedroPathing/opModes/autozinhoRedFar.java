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

@Autonomous(name = "Autozinho Red Far", group = "Main")
public class autozinhoRedFar extends OpMode {

    // Pedro Pathing & Timers
    private Follower follower;
    private Timer actionTimer = new Timer();
    private Timer shooterTimer = new Timer();
    private boolean shooterTimerStarted = false;
    private int pathState = 0;

    // Subsistemas e Sensores
    private DcMotor intake, indexer;
    private DcMotorEx leftShooter, rightShooter;
    private Servo angulator;
    private IntakeSensor intakeSensor;

    // Constantes do Shooter e Alvos
    private final double TARGET_VELOCITY = 2150;
    private final double VELOCITY_TOLERANCE = 100;

    // Definição das Poses (Posições do Campo)
    private final Pose startPose = new Pose(-42, -6, Math.toRadians(0));
    private final Pose shootPose = new Pose(-35, -7, Math.toRadians(-25));
    private final Pose intakePose = new Pose(-16, -6, Math.toRadians(-90));
    private final Pose intakeEnd = new Pose(-16, -37, Math.toRadians(-90));
    private final Pose intakeEndBack = new Pose(-16, -34, Math.toRadians(-90));

    // Posições Intermediárias de Coleta e Ré
    private final Pose intakeEndMed = new Pose(-26, -33, Math.toRadians(-90));
    private final Pose intakeEndMedBack = new Pose(-26, -29, Math.toRadians(-90)); // Posição para dar a ré

    // Poses das tentativas de coleta
    private final Pose collecting1 = new Pose(-18, -36, Math.toRadians(-90));

    // Caminhos (Paths)
    private Path goShoot1, goToIntake, goIntakeForward, goIntakeEndBackPath, goShoot2;
    private Path goToMedIntake, goToMedIntakeBack, goToColl1, goShootFinal;

    // Caminhos de repetição para voltar a coletar (2º Ciclo)
    private Path goToMedIntake2, goToMedIntakeBack2, goToColl1_2, goShootFinal2;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        initHardware();

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addLine("Robô Pronto para o Novo Autônomo!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (pathState != 2 && pathState != 6 && pathState != 13 && pathState != 17) {
            follower.update();
        }

        intakeSensor.periodic();
        autonomousPathUpdate();

        telemetry.addData("Estado Atual", pathState);
        telemetry.addData("Tem peça?", intakeSensor.hasArtifact() ? "SIM" : "NÃO");
        telemetry.addData("RPM do shooter", leftShooter.getVelocity());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            // --- 1º CICLO (Intake Inicial) ---
            case 0:
                follower.followPath(goShoot1);
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    actionTimer.resetTimer();
                    shooterTimerStarted = false;
                    pathState = 2;
                }
                break;
            case 2:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
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
                    follower.breakFollowing();
                    actionTimer.resetTimer();
                    shooterTimerStarted = false;
                    pathState = 6;
                }
                break;
            case 6:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
                    stopAll();
                    follower.followPath(goToMedIntake);
                    pathState = 7;
                }
                break;

            // --- 1ª TENTATIVA DE COLETA (Com Ré) ---
            case 7:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goToMedIntakeBack); // Robô dá a ré aqui
                    pathState = 75;
                }
                break;

            case 75:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goToColl1); // Segue para a coleta
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShootFinal);
                    pathState = 12;
                }
                break;

            // --- 1º DISPARO FINAL ---
            case 12:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    actionTimer.resetTimer();
                    shooterTimerStarted = false;
                    pathState = 13;
                }
                break;

            case 13:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
                    stopAll();
                    pathState = 145;
                }
                break;

            case 145:
                runIntakeWithSensor();
                if (actionTimer.getElapsedTimeSeconds() >= 2.4) {
                    pathState = 14;
                }
                break;

            // --- 2ª TENTATIVA DE COLETA (Repetição do Ciclo com Ré) ---
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(goToMedIntake2);
                    pathState = 140;
                }
                break;

            case 140:
                if (!follower.isBusy()) {
                    follower.followPath(goToMedIntakeBack2); // Robô dá a ré aqui também
                    pathState = 142;
                }
                break;

            case 142:
                if (!follower.isBusy()) {
                    follower.followPath(goToColl1_2); // Segue para a coleta
                    pathState = 15;
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShootFinal2);
                    pathState = 16;
                }
                break;

            // --- 2º DISPARO FINAL ---
            case 16:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    actionTimer.resetTimer();
                    shooterTimerStarted = false;
                    pathState = 17;
                }
                break;

            case 17:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2) {
                    stopAll();
                    pathState = 18; // Fim do autônomo
                }
                break;
        }
    }

    // --- MÉTODOS DOS SUBSISTEMAS ---
    private void runShooterSequence() {
        leftShooter.setVelocity(TARGET_VELOCITY);
        rightShooter.setVelocity(TARGET_VELOCITY);

        boolean isShooterReady = Math.abs(leftShooter.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE;

        if (!shooterTimerStarted) {
            shooterTimer.resetTimer();
            shooterTimerStarted = true;
        }

        if (isShooterReady || shooterTimer.getElapsedTimeSeconds() >= 4) {
            intake.setPower(1);
            indexer.setPower(-1);
        } else {
            indexer.setPower(0);
        }
    }

    private void runIntakeWithSensor() {
        intake.setPower(1);
        leftShooter.setPower(-.8);
        rightShooter.setPower(-.8);
        indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.4);
    }

    private void stopAll() {
        intake.setPower(0);
        indexer.setPower(0);
        shooterTimerStarted = false;
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

        // --- 1º Ciclo de Coleta com Ré ---
        goToMedIntake = new Path(new BezierLine(shootPose, intakeEndMed));
        goToMedIntake.setConstantHeadingInterpolation(intakeEndMed.getHeading());

        goToMedIntakeBack = new Path(new BezierLine(intakeEndMed, intakeEndMedBack));
        goToMedIntakeBack.setConstantHeadingInterpolation(intakeEndMedBack.getHeading());

        goToColl1 = new Path(new BezierLine(intakeEndMedBack, collecting1));
        goToColl1.setConstantHeadingInterpolation(collecting1.getHeading());
        goToColl1.setBrakingStrength(0.7);

        goShootFinal = new Path(new BezierLine(collecting1, shootPose));
        goShootFinal.setConstantHeadingInterpolation(shootPose.getHeading());

        // --- 2º Ciclo de Coleta com Ré (Duplicado) ---
        goToMedIntake2 = new Path(new BezierLine(shootPose, intakeEndMed));
        goToMedIntake2.setConstantHeadingInterpolation(intakeEndMed.getHeading());

        goToMedIntakeBack2 = new Path(new BezierLine(intakeEndMed, intakeEndMedBack));
        goToMedIntakeBack2.setConstantHeadingInterpolation(intakeEndMedBack.getHeading());

        goToColl1_2 = new Path(new BezierLine(intakeEndMedBack, collecting1));
        goToColl1_2.setConstantHeadingInterpolation(collecting1.getHeading());
        goToColl1_2.setBrakingStrength(0.7);

        goShootFinal2 = new Path(new BezierLine(collecting1, shootPose));
        goShootFinal2.setConstantHeadingInterpolation(shootPose.getHeading());
    }

    private void initHardware() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        angulator = hardwareMap.get(Servo.class, "angulator");
        intakeSensor = new IntakeSensor(hardwareMap);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angulator.setPosition(0.64);
    }
}