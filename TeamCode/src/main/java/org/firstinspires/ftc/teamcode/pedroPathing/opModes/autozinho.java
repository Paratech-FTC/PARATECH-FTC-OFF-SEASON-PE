package org.firstinspires.ftc.teamcode.pedroPathing.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//import org.firstinspires.ftc.teamcode.PointSystem.Intake;

@Autonomous(name = "autozinho teste", group = "Examples")
public class autozinho extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private Timer actionTimer;

//    private Intake intake;

    private int pathState = 0;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose forwardPose = new Pose(24, 0, Math.toRadians(0));
    private final Pose intakePose = new Pose(48, 24, Math.toRadians(90));

    private Path goForward;
    private Path goCurve;
    private Path goHome;

    public void buildPaths() {

        goForward = new Path(
                new BezierLine(
                        startPose,
                        forwardPose
                )
        );
        goForward.setLinearHeadingInterpolation(
                startPose.getHeading(),
                forwardPose.getHeading()
        );

        goCurve = new Path(
                new BezierCurve(
                        forwardPose,
                        new Pose(36, 0),
                        intakePose
                )
        );
        goCurve.setLinearHeadingInterpolation(
                forwardPose.getHeading(),
                intakePose.getHeading()
        );

        goHome = new Path(
                new BezierCurve(
                        intakePose,
                        new Pose(24, 12),
                        startPose
                )
        );
        goHome.setLinearHeadingInterpolation(
                intakePose.getHeading(),
                startPose.getHeading()
        );
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:

                follower.followPath(goForward);

                setPathState(1);

                break;

            case 1:

                if (!follower.isBusy()) {

                    follower.followPath(goCurve);

                    setPathState(2);
                }

                break;

            case 2:

                if (!follower.isBusy()) {

//                    intake.setPower(1);

                    actionTimer.resetTimer();

                    setPathState(3);
                }

                break;

            case 3:

                if (actionTimer.getElapsedTimeSeconds() >= 1.0) {

//                    intake.setPower(0);

                    follower.followPath(goHome);

                    setPathState(4);
                }

                break;

            case 4:

                if (!follower.isBusy()) {

                    setPathState(5);
                }

                break;

            case 5:

                break;
        }
    }

    public void setPathState(int state) {

        pathState = state;

        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        pathTimer = new Timer();
        actionTimer = new Timer();

//        intake = new Intake();
//        intake.init(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized");
    }

    @Override
    public void start() {

        setPathState(0);
    }

    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
