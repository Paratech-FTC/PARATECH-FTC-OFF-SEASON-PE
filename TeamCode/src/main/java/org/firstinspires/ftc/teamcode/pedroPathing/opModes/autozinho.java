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
    private final double TARGET_VELOCITY = 5700;
    private final double VELOCITY_TOLERANCE = 100;

    // Definição das Poses (Posições do Campo)
    private final Pose startPose  = new Pose(0, 0, Math.toRadians(0));
    private final Pose shootPose  = new Pose(20, 0, Math.toRadians(55)); // 45° virado para o goal esquerdo
    private final Pose intakePose = new Pose(0, 0, Math.toRadians(90));  // Posição para iniciar a coleta virado para esquerda
    private final Pose intakeEnd  = new Pose(0, 30, Math.toRadians(90)); // Avança 12 polegadas no eixo Y positivo (Para Frente do Robô)

    // Caminhos (Paths)
    private Path goShoot1, goToIntake, goIntakeForward, goShoot2;

    @Override
    public void init() {
        // Inicializa seguidor e hardware
        follower = Constants.createFollower(hardwareMap);
        initHardware();

        // Configura posição inicial e constrói trajetórias
        follower.setStartingPose(startPose);
        buildPaths();

        telemetry.addLine("Robô Pronto para o Autônomo!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Atualiza os dados do Pedro Pathing e do Sensor de Cor a cada ciclo
        follower.update();
        intakeSensor.periodic();

        // Executa a Máquina de Estados Principal
        autonomousPathUpdate();

        // Telemetria Limpa para Diagnóstico
        telemetry.addData("estado Atual", pathState);
        telemetry.addData("tem peça?", intakeSensor.hasArtifact() ? "SIM" : "NÃO");
        telemetry.addData("rpm do shooter", leftShooter.getVelocity());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0: // Passo 1: Anda 20 polegadas com spline apontando para o goal esquerdo (45°)
                follower.followPath(goShoot1);
                pathState = 1;
                break;

            case 1: // Aguarda chegar na posição de tiro
                if (!follower.isBusy()) {
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2: // Passo 2: Ativa o shooter por 3 segundos (respeitando o RPM pronto)
                runShooterSequence();
                if (actionTimer.getElapsedTimeSeconds() >= 3.0) {
                    stopAll();
                    follower.followPath(goToIntake); // Vai para a posição de alinhamento do Intake
                    pathState = 3;
                }
                break;

            case 3: // Aguarda chegar na pose (0, 0, 90°)
                if (!follower.isBusy()) {
                    // Passo 3: Inicia o trajeto para frente coletando a peça
                    follower.followPath(goIntakeForward);
                    pathState = 4;
                }
                break;

            case 4: // Passo 4: Executa o movimento de Intake Inteligente (baseado no sensor de cor)
                runIntakeWithSensor();

                // Se o trajeto terminar ou o sensor detectar que a peça já entrou, finaliza a coleta
                if (!follower.isBusy() || intakeSensor.hasArtifact()) {
                    stopAll();
                    follower.followPath(goShoot2); // Volta para a posição de tiro
                    pathState = 5;
                }
                break;

            case 5: // Aguarda retornar à posição de disparo
                if (!follower.isBusy()) {
                    pathState = 6;
                }
                break;

            case 6: // Passo 5: Ativa o shooter novamente com a nova peça coletada
                runShooterSequence();
                break;
        }
    }

    // --- MÉTODOS DOS SUBSISTEMAS ---
    private void runShooterSequence() {
        leftShooter.setVelocity(TARGET_VELOCITY);
        rightShooter.setVelocity(TARGET_VELOCITY);

        boolean isShooterReady = Math.abs(leftShooter.getVelocity() - TARGET_VELOCITY) < VELOCITY_TOLERANCE;

        if (isShooterReady) {
            intake.setPower(1.0);
            indexer.setPower(-0.7); // Puxa para dentro da câmara de disparo
        } else {
            indexer.setPower(0);    // Segura a peça até atingir a velocidade
        }
    }

    private void runIntakeWithSensor() {
        intake.setPower(1.0);
        indexer.setPower(intakeSensor.hasArtifact() ? 0 : -0.7);
    }

    private void stopAll() {
        intake.setPower(0);
        indexer.setPower(0);
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);
    }

    private void buildPaths() {
        // Trajeto 1: De (0,0,0) até a Posição de Disparo (20,0) interpolando para 45° (Goal)
        goShoot1 = new Path(new BezierLine(startPose, shootPose));
        goShoot1.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        // Trajeto 2: Sai do disparo e vai se alinhar em (0,0) olhando para a esquerda (90°)
        goToIntake = new Path(new BezierCurve(shootPose, new Pose(10, 0), intakePose));
        goToIntake.setLinearHeadingInterpolation(shootPose.getHeading(), intakePose.getHeading());

        // Trajeto 3: Como o robô está em 90° (olhando para a esquerda do campo), andar "para frente" significa
        // subir no eixo Y positivo. O robô vai de (0,0) para (0,12) mantendo fixo o heading em 90°.
        goIntakeForward = new Path(new BezierLine(intakePose, intakeEnd));
        goIntakeForward.setConstantHeadingInterpolation(intakePose.getHeading());

        // Trajeto 4: Retorna da posição final da coleta direto para o ponto de disparo reajustando o ângulo para 45°
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

        angulator.setPosition(0.35); // Posição padrão segura
    }
}