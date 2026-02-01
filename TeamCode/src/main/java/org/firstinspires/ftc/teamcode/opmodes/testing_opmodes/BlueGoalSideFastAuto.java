package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.opmodes.NGAutoOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;

@Autonomous
public class BlueGoalSideFastAuto extends NGAutoOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PoseStorage.resetPose();

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.x.value() < -30.0) {
                return new MinMax(-5,5);
            } else {
                return new MinMax(-120,120);
            }
        };

/*TrajectoryActionBuilder PathToGate = collectFirstSet.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-7, -44.5), Math.toRadians(270))
                .lineToY(-56, new TranslationalVelConstraint(12));
                    new SleepAction(2);*/

        Pose2d beginPose = new Pose2d(-52, -48, Math.toRadians(235));
        initAuto(beginPose);

        TrajectoryActionBuilder moveBackwardPath = drive.actionBuilder(beginPose)
                .afterTime(1.2, intake2_0.transferUsingRollersForTime(0.8, 1))
                .lineToY(-27, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-150, 150));

        TrajectoryActionBuilder ToFirstSet = moveBackwardPath.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-20, -21, Math.toRadians(270)), Math.toRadians(5), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-60, 60))
                .afterTime(0, intake2_0.collect(1))
                .splineToLinearHeading(new Pose2d(-20, -48.2, Math.toRadians(270)), Math.toRadians(-100), new TranslationalVelConstraint(50), new ProfileAccelConstraint(-20, 30))
                .splineToSplineHeading(new Pose2d(-26, -16, Math.toRadians(206)), Math.toRadians(100), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-70, 70));

        TrajectoryActionBuilder ToSecondSet = ToFirstSet.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(24, -8, Math.toRadians(305)), Math.toRadians(-20), new TranslationalVelConstraint(80),  new ProfileAccelConstraint(-120, 120))
                .afterTime(0, intake2_0.collect(1))
                .splineToSplineHeading(new Pose2d(24, -48, Math.toRadians(310)), Math.toRadians(-60), new TranslationalVelConstraint(50))
                .splineToSplineHeading(new Pose2d(-24, -15, Math.toRadians(230)), Math.toRadians(-50), new TranslationalVelConstraint(60));

        TrajectoryActionBuilder ToFirstGateSet = ToSecondSet.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(11, -53, Math.toRadians(236)), Math.toRadians(-120), new TranslationalVelConstraint(40));

        TrajectoryActionBuilder ToGoal1 = ToFirstGateSet.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(11, -20, Math.toRadians(240)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(1.0, intake2_0.transferUsingRollersForTime(1, 1))
                .splineToSplineHeading(new Pose2d(-52, -15, Math.toRadians(225)), Math.toRadians(-150), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(9, -47.6, Math.toRadians(245)), Math.toRadians(-120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(9, -52));

        TrajectoryActionBuilder ToGoal2 = ToGoal1.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(11, -20, Math.toRadians(240)), Math.toRadians(90), new TranslationalVelConstraint(80))
                .afterTime(1.0, intake2_0.transferUsingRollersForTime(1.2, 1))
                .splineToSplineHeading(new Pose2d(-52, -15, Math.toRadians(225)), Math.toRadians(-150), new TranslationalVelConstraint(60), smartScore)
                .splineToLinearHeading(new Pose2d(9, -47.6, Math.toRadians(245)), Math.toRadians(-120), new TranslationalVelConstraint(40))
                .strafeToConstantHeading(new Vector2d(9, -52));

        TrajectoryActionBuilder ToGoal3 = ToGoal2.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(7, -35, Math.toRadians(240)), Math.toRadians(90), new TranslationalVelConstraint(60))
                .afterTime(1, intake2_0.transferUsingRollersForTime(1.2, 1))
                .splineToSplineHeading(new Pose2d(-52, -15, Math.toRadians(225)), Math.toRadians(-155), new TranslationalVelConstraint(60),  new ProfileAccelConstraint(-120, 120));

        TrajectoryActionBuilder leaveFromGate = ToGoal2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(10, -36), new TranslationalVelConstraint(120));

        telemetry.addLine("Ready To Start");
        telemetry.update();

        Action scorePreLoaded = moveBackwardPath.build();
        Action intakeFirstSet = ToFirstSet.build();
        Action intakeSecondSet = ToSecondSet.build();
        Action toGateSet1 = ToFirstGateSet.build();
        Action ShootToIntake1 = ToGoal1.build();
        Action shootToIntake2 = ToGoal2.build();
        Action shoot3 = ToGoal3.build();
        Action leave = leaveFromGate.build();

        telemetry.addLine("Paths Built");
        telemetry.update();

        double shooterTargetVel = DECODERobotConstants.closeZoneShootingVel;
        double hoodShootingPos = DECODERobotConstants.closeShootPos;

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        bulkRead.update(),
                        intake2_0.updateFlywheelPID(),
                        new ParallelAction(
                                intake2_0.runShooter(shooterTargetVel, 30),
                                new SequentialAction(
                                        scorePreLoaded,
                                        new SequentialAction(
                                                intakeFirstSet,
                                                intake2_0.transferUsingRollersForTime(1, 1)
                                        ),
                                        new SequentialAction(
                                                intakeSecondSet,
                                                intake2_0.transferUsingRollersForTime(1, 1)
                                        ),
                                        new SequentialAction(
                                                toGateSet1,
                                                intake2_0.collect(2)
                                        ),
                                        new SequentialAction(ShootToIntake1,
                                                intake2_0.collect(2),
                                                shootToIntake2
                                        ),
                                        new SequentialAction(
                                                intake2_0.collect(2),
                                                new Action() {
                                                    private boolean initialized = false;
                                                    private Action conditionalSequence = null;

                                                    @Override
                                                    public boolean run(@NonNull TelemetryPacket packet) {
                                                        if (!initialized) {

                                                            if (getRuntime() < 27.6) {
                                                                conditionalSequence = new SequentialAction(
                                                                        shoot3
                                                                );
                                                            }else if(getRuntime() > 27.6){
                                                                conditionalSequence = new SequentialAction(
                                                                        leave
                                                                );
                                                            }
                                                            initialized = true;
                                                        }

                                                        if (conditionalSequence != null) {
                                                            return conditionalSequence.run(packet);
                                                        }
                                                        return false;
                                                    }


                                        }
                                )
                        )
                )
        )
        );
        }
    }