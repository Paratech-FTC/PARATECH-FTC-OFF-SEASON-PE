package org.firstinspires.ftc.teamcode.pedroPathing.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem {
    private Follower follower;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Constantes do PID
    private final double kP = 1.4;
    private final double kD = 0.6;
    private final double headingDeadzone = Math.toRadians(1.5);

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
    }

    public void update() { follower.update(); }

    public Pose getPose() { return follower.getPose(); }

    public void drive(double forward, double strafe, double turn) {
        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    // A lógica de alinhamento agora vive aqui
    public double calculateAlignment(Pose robotPose, Pose target) {
        double targetHeading = Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX());
        double error = normalizeAngle(targetHeading - robotPose.getHeading());

        double dt = timer.seconds();
        timer.reset();

        double derivative = (error - lastError) / dt;
        lastError = error;

        if (Math.abs(error) < headingDeadzone) return 0;
        return Math.max(-1, Math.min(1, (error * kP) + (derivative * kD)));
    }

    private double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}