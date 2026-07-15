package org.firstinspires.ftc.teamcode.pedroPathing.opModes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.subsystems.*;

@TeleOp(name = "Main TeleOp")
public class Main extends LinearOpMode {
    //Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private AngulatorSubsystem angulator;

    //Estados
    private boolean lastA = false;
    private double headingOffset = 0;
    private boolean aimAlign = false;
    private Pose currentTarget = null;

    private final Pose leftGoal = new Pose(40, 40);
    private final Pose rightGoal = new Pose(40, -40);

    @Override
    public void runOpMode() {
        //Inicialização
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        angulator = new AngulatorSubsystem(hardwareMap);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            intake.update();
            Pose robotPose = drive.getPose();

            //Lógica de Alvo
            if (gamepad1.x) { currentTarget = leftGoal; aimAlign = true; }
            if (gamepad1.b) { currentTarget = rightGoal; aimAlign = true; }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) aimAlign = false;

            if (currentTarget != null) {
                angulator.updateByDistance(robotPose.distanceFrom(currentTarget));
            }

            //Movimentação
            if (gamepad1.a && !lastA) headingOffset = robotPose.getHeading();
            lastA = gamepad1.a;

            double fieldHeading = robotPose.getHeading() - headingOffset;
            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;

            //Translação relativa ao campo
            double x = forward * Math.cos(fieldHeading) - strafe * Math.sin(fieldHeading);
            double y = forward * Math.sin(fieldHeading) + strafe * Math.cos(fieldHeading);

            //Align PD (DriveSubsystem)
            double turn = (aimAlign && currentTarget != null)
                    ? drive.calculateAlignment(robotPose, currentTarget)
                    : -gamepad1.right_stick_x;

            drive.drive(x, y, turn);

            //Shooter e Intake
            if (gamepad1.y) {
                shooter.setShooterPower(shooter.targetVelocity);
                intake.runIndexer((gamepad1.right_bumper && shooter.isReady()) ? -0.7 : 0);
            } else {
                shooter.setShooterPower(0);
                intake.runIndexer(0);
            }

            if (gamepad1.right_bumper) {
                intake.runIntake(1.0);
                if (!gamepad1.y) intake.runIndexer(intake.hasPiece() ? 0 : -0.7);
            } else if (gamepad1.left_bumper) {
                intake.runIntake(-1.0);
                intake.runIndexer(1.0);
            } else if (!gamepad1.y) {
                intake.runIntake(0);
            }

            //Presets Angulator
            if (gamepad1.dpad_down) angulator.setPosition(1.0);
            if (gamepad1.dpad_right) angulator.setPosition(0.5);
            if (gamepad1.dpad_up) angulator.setPosition(0.0);

            telemetry.addData("Shooter Ready", shooter.isReady());
            telemetry.addData("Aim Align", aimAlign);
            telemetry.update();
        }
    }
}