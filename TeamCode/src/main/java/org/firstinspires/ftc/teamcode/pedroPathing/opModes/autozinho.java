package org.firstinspires.ftc.teamcode.pedroPathing.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "autozinho teste", group = "Main")
public class autozinho extends OpMode {

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
    private final double TARGET_VELOCITY = 2200;
    private final double VELOCITY_TOLERANCE = 100;

    // Definição das Poses (Posições do Campo)
    private final Pose startPose  = new Pose(39, 30, Math.toRadians(50));
    private final Pose shootPose  = new Pose(16, 2, Math.toRadians(55));
    private final Pose intakePose = new Pose(0, 0, Math.toRadians(90));
    private final Pose intakeEnd  = new Pose(16, 35, Math.toRadians(90));

    // Caminhos (Paths)
    private Path goShoot1, goToIntake, goIntakeForward, goShoot2;

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
        telemetry.addData("RPM do Shooter", leftShooter.getVelocity());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Dispara o primeiro trajeto para oAlvo
                follower.followPath(goShoot1);
                pathState = 1;
                break;

            case 1: // Aguarda chegar na posição de tiro. DICA: Já ligamos o shooter um pouco antes se quiser, ou aqui.
                // Se quiser ligar o shooter ANTES de chegar, basta colocar leftShooter.setVelocity(TARGET_VELOCITY) aqui.
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2: // Ativa o shooter, espera o RPM e dispara por 3 segundos
                runShooterSequence(); // Mantém o shooter girando

                // Verifica se atingiu a velocidade e o timer de 3s estourou
                if (isShooterAtSpeed() && actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    stopShooterAndMechanisms();
                    follower.followPath(goToIntake); // Vai para o intake
                    pathState = 3;
                }
                break;

            case 3: // Vai para a pose de alinhamento do intake
                if (!follower.isBusy()) {
                    follower.followPath(goIntakeForward);
                    pathState = 4;
                }
                break;

            case 4: // Executa a coleta com sensor
                runIntakeWithSensor();

                if (!follower.isBusy() || intakeSensor.hasArtifact()) {
                    stopIntake(); // Para o intake para guardar a peça

                    // AQUI ESTÁ O TRUQUE: Já mandamos o shooter ligar ENQUANTO o robô faz o caminho de volta (goShoot2)
                    leftShooter.setVelocity(TARGET_VELOCITY);
                    rightShooter.setVelocity(TARGET_VELOCITY);

                    follower.followPath(goShoot2); // Retorna para a posição de tiro
                    pathState = 5;
                }
                break;

            case 5: // Aguarda retornar à posição de disparo (com o shooter já aquecendo no caminho!)
                // Mantém o shooter girando no talo enquanto viaja de volta
                leftShooter.setVelocity(TARGET_VELOCITY);
                rightShooter.setVelocity(TARGET_VELOCITY);

                if (!follower.isBusy()) {
                    actionTimer.resetTimer(); // Reseta o timer para contar os 3 segundos de tiro
                    pathState = 6;
                }
                break;

            case 6: // Segundo Disparo
                runShooterSequence(); // Cuida do RPM e joga a peça com o indexer

                if (isShooterAtSpeed() && actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    stopAll();
                    // SE VOCÊ FOR ADICIONAR MAIS CICLOS, BASTA MUDAR O ESTADO AQUI:
                    // pathState = 7; (e criar os próximos casos para o próximo intake e tiro)
                }
                break;
        }
    }

    // --- MÉTODOS AUXILIARES E DOS SUBSISTEMAS ---

    // Retorna verdadeiro se o shooter estiver na velocidade correta
    private boolean isShooterAtSpeed() {
        return Math.abs(leftShooter.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE;
    }

    // Liga o motor do shooter e, se estiver no RPM certo, ativa o indexer/intake para atirar
    private void runShooterSequence() {
        leftShooter.setVelocity(TARGET_VELOCITY);
        rightShooter.setVelocity(TARGET_VELOCITY);

        if (isShooterAtSpeed()) {
            intake.setPower(1.0);
            indexer.setPower(-0.7);
        } else {
            indexer.setPower(0);    // Aguarda o motor estabilizar antes de empurrar a peça
        }
    }

    private void runIntakeWithSensor() {
        intake.setPower(1.0);
        indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.7);
    }

    private void stopShooterAndMechanisms() {
        intake.setPower(0);
        indexer.setPower(0);
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    private void stopIntake() {
        intake.setPower(0);
        indexer.setPower(0);
    }

    private void stopAll() {
        intake.setPower(0);
        indexer.setPower(0);
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    private void buildPaths() {
        goShoot1 = new Path(new BezierLine(startPose, shootPose));
        goShoot1.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        goToIntake = new Path(new BezierCurve(shootPose, new Pose(10, 0), intakePose));
        goToIntake.setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading());

        goIntakeForward = new Path(new BezierLine(intakePose, intakeEnd));
        goIntakeForward.setConstantHeadingInterpolation(intakePose.getHeading());

        goShoot2 = new Path(new BezierCurve(intakeEnd, new Pose(10, 6), shootPose));
        goShoot2.setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPose.getHeading());
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
        angulator.setPosition(0.8);
    }
}