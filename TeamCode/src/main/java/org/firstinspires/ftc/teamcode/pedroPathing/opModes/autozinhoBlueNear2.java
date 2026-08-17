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

@Autonomous(name = "Autozinho Blue Near 2", group = "Main")
public class autozinhoBlueNear2 extends OpMode {

    // Pedro Pathing & Timers
    private Follower follower;
    private Timer actionTimer = new Timer();
    private int pathState = 0;

    // Subsistemas e Sensores
    private DcMotor intake, indexer;
    private DcMotorEx leftShooter, rightShooter;
    private Servo angulator;
    private IntakeSensor intakeSensor;

    // Constantes do Shooter e Alvos
    private final double TARGET_VELOCITY = 1800;
    private final double VELOCITY_TOLERANCE = 130;

    // Definição das Poses (Posições do Campo)
    private final Pose startPose = new Pose(39, 30, Math.toRadians(50));
    private final Pose shootPose = new Pose(16, 4, Math.toRadians(52));
    private final Pose waitSecondIntake = new Pose(11, 36, Math.toRadians(90));

    // Intake Normal
    private final Pose intakePose = new Pose(0, 7, Math.toRadians(90));
    private final Pose intakeEnd = new Pose(0, 38, Math.toRadians(90));
    private final Pose intakeEndBack = new Pose(0, 28, Math.toRadians(90));

    // Novo Intake (substituindo o gate)
    private final Pose newIntakePose = new Pose(-16, 7, Math.toRadians(90));
    private final Pose newIntakeEnd = new Pose(-16, 41, Math.toRadians(90));
    private final Pose newIntakeEndBack = new Pose(-16, 35, Math.toRadians(90));

    // Caminhos (Paths)
    private Path
            goShoot1, goToIntake, goIntakeForward, goIntakeEndBackPath, goShoot2,
            goToNewIntake, goNewIntakeForward, goNewIntakeEndBackPath, goShoot3,
            goToFileira2, goShoot4;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        initHardware();

        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addLine("Robô Pronto para o Autônomo!");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        intakeSensor.periodic();

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
                    follower.followPath(goToNewIntake);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(goNewIntakeForward);
                    pathState = 8;
                }
                break;

            case 8:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goNewIntakeEndBackPath);
                    pathState = 85;
                }
                break;

            case 85:
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    stopAll();
                    follower.followPath(goShoot3);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 10;
                }
                break;

            case 10:
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 2.0) {
                    stopAll();
                    pathState = 105;
                }
                break;

            case 105: // Inicia o caminho para a fileira
                runIntakeWithSensor();
                if (!follower.isBusy()) {
                    follower.followPath(goToFileira2);
                    pathState = 106; // Vai para o novo estado de confirmação da peça
                }
                break;

            case 106: // <--- NOVO: Mantém o intake rodando até o sensor pegar a peça ou o robô estabilizar
                runIntakeWithSensor();
                // Se a peça foi detectada OU se o robô já terminou de se posicionar, podemos prosseguir
                if (intakeSensor.hasArtifact() || !follower.isBusy()) {
                    stopAll(); // Para os motores de coleta/indexação com a peça segura
                    follower.followPath(goShoot4);
                    pathState = 12; // Pula direto para o tiro
                }
                break;

            case 12:
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

    private void buildPaths() {
        goShoot1 = new Path(new BezierLine(startPose, shootPose));
        goShoot1.setConstantHeadingInterpolation(shootPose.getHeading());

        goToIntake = new Path(new BezierLine(shootPose, intakePose));
        goToIntake.setConstantHeadingInterpolation(intakePose.getHeading());

        goIntakeForward = new Path(new BezierLine(intakePose, intakeEnd));
        goIntakeForward.setConstantHeadingInterpolation(intakePose.getHeading());
        goIntakeForward.setBrakingStrength(0.4);

        goIntakeEndBackPath = new Path(new BezierLine(intakeEnd, intakeEndBack));
        goIntakeEndBackPath.setConstantHeadingInterpolation(intakeEndBack.getHeading());

        goShoot2 = new Path(new BezierLine(intakeEndBack, shootPose));
        goShoot2.setConstantHeadingInterpolation(shootPose.getHeading());

        goToNewIntake = new Path(new BezierLine(shootPose, newIntakePose));
        goToNewIntake.setConstantHeadingInterpolation(newIntakePose.getHeading());

        goNewIntakeForward = new Path(new BezierLine(newIntakePose, newIntakeEnd));
        goNewIntakeForward.setConstantHeadingInterpolation(newIntakePose.getHeading());
        goNewIntakeForward.setBrakingStrength(0.4);

        goNewIntakeEndBackPath = new Path(new BezierLine(newIntakeEnd, newIntakeEndBack));
        goNewIntakeEndBackPath.setConstantHeadingInterpolation(newIntakeEndBack.getHeading());

        goShoot3 = new Path(new BezierLine(newIntakeEndBack, shootPose));
        goShoot3.setConstantHeadingInterpolation(shootPose.getHeading());

        goToFileira2 = new Path(new BezierLine(shootPose, waitSecondIntake));
        goToFileira2.setConstantHeadingInterpolation(waitSecondIntake.getHeading());
        goToFileira2.setBrakingStrength(0.4);

        goShoot4 = new Path(new BezierLine(waitSecondIntake, shootPose));
        goShoot4.setConstantHeadingInterpolation(shootPose.getHeading());
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
        angulator.setPosition(0.70);
    }
}