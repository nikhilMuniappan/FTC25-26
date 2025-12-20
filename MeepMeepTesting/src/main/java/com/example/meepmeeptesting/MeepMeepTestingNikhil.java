package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.awt.Color;
public class MeepMeepTestingNikhil {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity sampleBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity specimenBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,0,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,100);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,200);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,0,100);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0,0,185);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,50,255);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,50,255);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return  new Color(0,80,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,80,255);
                    }

                })
                .build();
        RoadRunnerBotEntity sampleBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorScheme() {
                    @NotNull
                    @Override
                    public Color getUI_MAIN_BG() {
                        return new Color(0,100,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TEXT_COLOR() {
                        return new Color(0,0,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_FG() {
                        return new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_SLIDER_BG() {
                        return new Color(255,255,255);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_MARKER_COLOR() {
                        return new Color(0,100,20);
                    }

                    @Override
                    public boolean isDark() {
                        return false;
                    }

                    @NotNull
                    @Override
                    public Color getBOT_BODY_COLOR() {
                        return new Color(0,155,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_WHEEL_COLOR() {
                        return new Color(0,50,0);
                    }

                    @NotNull
                    @Override
                    public Color getBOT_DIRECTION_COLOR() {
                        return new Color(0, 100, 0);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_X_COLOR() {
                        return new Color(0,200,50);
                    }

                    @NotNull
                    @Override
                    public Color getAXIS_Y_COLOR() {
                        return new Color(0,200,50);
                    }

                    @Override
                    public double getAXIS_NORMAL_OPACITY() {
                        return 0.7;
                    }

                    @Override
                    public double getAXIS_HOVER_OPACITY() {
                        return 0.3;
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_PATH_COLOR() {
                        return  new Color(0,255,0);
                    }

                    @NotNull
                    @Override
                    public Color getTRAJECTORY_TURN_COLOR() {
                        return new Color(0,255,0);
                    }

                })
                .build();
        AccelConstraint highMode = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -20.0) {
                return new MinMax(-10,50);
            } else {
                return new MinMax(-30,50);
            }
        };
        Pose2d basket = new Pose2d(-55, -55, Math.toRadians(45));

        AccelConstraint smartScore = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -32.0) {
                return new MinMax(-20,80);
            } else {
                return new MinMax(-30,80);
            }
        };
        AccelConstraint intakeAccel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -42.0) {
                return new MinMax(-10,22);
            } else {
                return new MinMax(-30,50);
            }
        };
        VelConstraint intakeVel = (robotPose, _path, _disp) -> {
            if (robotPose.position.y.value() < -25.0) {
                return 20;
            } else {
                return 50;
            }
        };
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(64, -36 , Math.toRadians(-90)))
                .lineToY(-5)
                .setReversed(true)
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(-48, -24, Math.toRadians(0)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.2)
                //.lineToX(-35)
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
                        //.turn(-0.2)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-31, -11, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.2)
                //.lineToX(-35)
                .setTangent(-90)
                .splineToSplineHeading(new Pose2d(-60, -48.5, Math.toRadians(70)), Math.toRadians(240))
                        //.turn(-0.2)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(-29, -11, Math.toRadians(0)), Math.toRadians(0))
                .build());
         */
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-56, -44 , Math.toRadians(55)))
                .lineToY(36)
                    .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(10))
                       // .waitSeconds(0.00001)
                .lineToYConstantHeading(52, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55))
                        .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(-90)), Math.toRadians(5), new TranslationalVelConstraint(60))
                //.splineToSplineHeading(new Pose2d(11, -30, Math.toRadians(0)), Math.toRadians(0))
                //    .waitSeconds(0.1)
                //.strafeToSplineHeading(new Vector2d(-52, -34), Math.toRadians(65))
                .build());*/
        /*sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(61, 9, Math.toRadians(0)))
                .splineToSplineHeading(new Pose2d(-50, 36, Math.toRadians(-55)), Math.toRadians(50), new TranslationalVelConstraint(80))
                    .waitSeconds(4)
                .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(-90)), Math.toRadians(40), new TranslationalVelConstraint(60))
                    //.waitSeconds(0.2)
                .lineToY(53, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, 36), Math.toRadians(-55), new TranslationalVelConstraint(80))
                        .waitSeconds(4)

                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(90)), Math.toRadians(-20), new TranslationalVelConstraint(60))
                        .lineToY(-53, new TranslationalVelConstraint(15))
                .strafeToSplineHeading(new Vector2d(-50, -36), Math.toRadians(55), new TranslationalVelConstraint(80))
                .build())
*/
        sampleBot.runAction(sampleBot.getDrive().actionBuilder(new Pose2d(-25, -10 , Math.toRadians(55)))
                //.strafeToSplineHeading(new Vector2d(-22, -5), Math.toRadians(55), new TranslationalVelConstraint(50))
                //.splineToLinearHeading(new Pose2d(35, -24, Math.toRadians(90)), Math.toRadians(3), new TranslationalVelConstraint(50))
                /*.splineToConstantHeading(new Vector2d(-4, -43.5), Math.toRadians(-90))
                        .lineToY(-56)*/
                //.splineToLinearHeading(new Pose2d(11, 17, Math.toRadians(-90)), Math.toRadians(-20), new TranslationalVelConstraint(40))
                        .strafeToLinearHeading(new Vector2d(54, -10), Math.toRadians(35))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(sampleBot)
                .start();
    }
}
