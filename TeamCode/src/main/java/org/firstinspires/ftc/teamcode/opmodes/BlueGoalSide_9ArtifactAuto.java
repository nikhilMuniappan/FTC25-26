package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;

@Autonomous
public class BlueGoalSide_9ArtifactAuto extends NGAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-52, -48 , Math.toRadians(55));
        initAuto(beginPose);

        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10, 50);
            } else {
                return new MinMax(-30, 50);
            }
        };

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-20, 80);
            } else {
                return new MinMax(-30, 80);
            }
        };

        VelConstraint highModeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .lineToY(-10, new TranslationalVelConstraint(60));
        TrajectoryActionBuilder PathToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-15, -24), Math.toRadians(90));
        TrajectoryActionBuilder collectFirstSet = drive.actionBuilder(new Pose2d(-12, -33, Math.toRadians(90)))
                .lineToY(-56, new TranslationalVelConstraint(10));
        TrajectoryActionBuilder PathToGoal = collectFirstSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-22, -5), Math.toRadians(55), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder PathToSecondSet = PathToGoal.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(40));
        TrajectoryActionBuilder collectSecondSet = drive.actionBuilder(new Pose2d(10, -34, Math.toRadians(90)))
                .lineToY(-55, new TranslationalVelConstraint(10));
        TrajectoryActionBuilder PathToGoal2 = collectSecondSet.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-22, -5), Math.toRadians(55), new TranslationalVelConstraint(50));
        TrajectoryActionBuilder leaveLaunchLine = PathToGoal2.endTrajectory().fresh()
                        .strafeToConstantHeading(new Vector2d(5, -10), new TranslationalVelConstraint(70));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action goToFirstSet = PathToFirstSet.build();
        Action CollectFirstSet = collectFirstSet.build();
        Action goToGoal = PathToGoal.build();
        Action goToSecondSet = PathToSecondSet.build();
        Action CollectSecondSet = collectSecondSet.build();
        Action goToGoal2 = PathToGoal2.build();
        Action leave = leaveLaunchLine.build();

        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.closeShootingVel;
        double hoodShootingPos = DECODERobotConstants.farShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                    bulkRead.update(),
                    new SequentialAction(
                            new ParallelAction(
                                    intake2_0.runShooter(shooterTargetVel, 6),
                                    intake2_0.setHoodAdjuster(hoodShootingPos),
                                    scorePreLoaded
                            ),
                            new ParallelAction(
                                    intake2_0.shoot(4.5, shooterTargetVel),
                                    intake2_0.transferUsingRollersForTime(5, 1)
                            ),
                            new SequentialAction(
                                    new SequentialAction(
                                    intake2_0.disableTransfer()
                                    ),
                                    goToFirstSet
                            ),
                            new ParallelAction(
                                intake2_0.collect(2.5),
                                CollectFirstSet
                            ),
                            new ParallelAction(
                                goToGoal,
                                    intake2_0.runShooter(shooterTargetVel, 6),
                                    intake2_0.setHoodAdjuster(hoodShootingPos)
                            ),
                            new ParallelAction(
                                    intake2_0.transferUsingRollersForTime(5, 1),
                                    intake2_0.shoot(4.5, shooterTargetVel)
                            ),
                            new ParallelAction(
                                    new SequentialAction(
                                    intake2_0.disableTransfer()
                                    ),
                                    goToSecondSet
                            ),
                            new ParallelAction(
                                    intake2_0.collect(2.5),
                                    CollectSecondSet
                            ),
                            new ParallelAction(
                                    goToGoal2,
                                        intake2_0.runShooter(shooterTargetVel, 6),
                                        intake2_0.setHoodAdjuster(hoodShootingPos)
                            ),
                            new ParallelAction(
                                    intake2_0.transferUsingRollersForTime(5, 1),
                                    intake2_0.shoot(4.5, shooterTargetVel)
                            ),
                            new ParallelAction(
                                    intake2_0.disableTransfer(),
                                    intake2_0.stopShooter()
                            ),
                            new SequentialAction(
                                    leave
                            )
                    )
                )
        );
    }
}
