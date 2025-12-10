package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DecodeCAM;
import org.firstinspires.ftc.teamcode.Pose2d;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;

import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ATeleOp_FusedControl extends LinearOpMode{
    MecaTank mecaTank;
    private DecodeCAM camera;
    ElapsedTime timer;
    private NGMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rollers;
    private NGMotor flywheels;
    private DcMotor transferRollers;
    private Servo hoodAdjuster;

    private enum Alliance {
        BLUE,
        RED
    }
    private Alliance currentAlliance = Alliance.BLUE;

    private static final double GOAL_X = -58.3727f;
    private static final double BLUE_GOAL_Y = -55.6425f;
    private static final double RED_GOAL_Y = 55.6425f;

    private double targetGoalY = BLUE_GOAL_Y;

    private boolean autoAimActive = false;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        rollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.rollers);
        transferRollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.transferRollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
        mecaTank = new MecaTank(hardwareMap, telemetry, timer);
        camera = new DecodeCAM();

        mecaTank.setPoseEstimate(new Pose2d(12.0, 12.0, Math.toRadians(0)));

        camera.init(hardwareMap.appContext, hardwareMap, telemetry);

        transferRollers.setDirection(DcMotor.Direction.REVERSE);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hoodAdjuster.setPosition(DECODERobotConstants.hoodStartPos);

        flywheels.setCustomVelocityPID(0.0, 0.008, 0.015, 0.0001, 0.000426);

        double hoodPos = hoodAdjuster.getPosition();
        boolean lastUp = false;
        boolean lastDown = false;

        double shooterCloseVel = DECODERobotConstants.closeShootingVel;
        double shooterFarVel = DECODERobotConstants.farShootingVel;
        double hoodClosePos = DECODERobotConstants.closeShootPos;
        double hoodFarPos = DECODERobotConstants.farShootPos;
        boolean shootReady = false;
        boolean flywheelsActive = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                currentAlliance = Alliance.RED;
                targetGoalY = RED_GOAL_Y;
            } else if (gamepad1.dpad_right) {
                currentAlliance = Alliance.BLUE;
                targetGoalY = BLUE_GOAL_Y;
            }

            telemetry.addData("INIT Status", "READY");
            telemetry.addData("Alliance (D-PAD L/R)", currentAlliance);
            telemetry.addData("Target Goal (X, Y)", "%.2f, %.2f", GOAL_X, targetGoalY);
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        long startTime = System.currentTimeMillis();
        boolean poseAcquired = false;

        while (opModeIsActive() && !poseAcquired && (System.currentTimeMillis() < startTime + 5000)) {
            Pose2d absoluteCameraPose = camera.getAbsoluteRobotPose();

            if (absoluteCameraPose != null) {
                mecaTank.setPoseEstimate(absoluteCameraPose);
                poseAcquired = true;
                telemetry.addLine("POSE ACQUIRED from Camera. Full Control Ready.");
            } else {
                mecaTank.updateAutoAlign();
                telemetry.addLine("Seeking Pose... Move robot slightly to acquire AprilTag.");
            }
            telemetry.update();
            sleep(20);
        }

        if (!poseAcquired) {
            telemetry.addLine("Warning: Camera pose not acquired. Relying on odometry from default/Auto end pose.");
            telemetry.update();
        }

        while (!isStopRequested() && opModeIsActive()) {
            mecaTank.updateAutoAlign();
            flywheels.updateFlywheels();

            double strafeInput = gamepad1.left_stick_x;
            double forwardInput = -gamepad1.left_stick_y;
            double manualTurnInput = gamepad1.right_stick_x;

            boolean hoodUp = gamepad2.dpad_up;
            boolean hoodDown = gamepad2.dpad_down;
            //telemetry.addData("hoodAdjuster Position: ", hoodAdjuster.getPosition());
            //telemetry.addData("Flywheel Vel: ", flywheels.getVelocity());
            //telemetry.update();

            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                autoAimActive = true;
            } else if (gamepad1.y) {
                autoAimActive = false;
            }

            Pose2d currentPose = mecaTank.getPoseEstimate();

            if (autoAimActive) {

                double targetHeading_rad = Math.atan2(
                        targetGoalY - currentPose.getY(),
                        GOAL_X - currentPose.getX()
                );
                double finalTurnPower = mecaTank.calculateAutoTurnPower(
                        Math.toDegrees(targetHeading_rad),
                        Math.toDegrees(currentPose.getHeading())
                );
                mecaTank.driveFieldCentric(strafeInput, forwardInput, finalTurnPower, Math.toDegrees(currentPose.getHeading()));

                double deltaX = GOAL_X - currentPose.getX();
                double deltaY = targetGoalY - currentPose.getY();
                double distanceToGoal = Math.hypot(deltaX, deltaY);

                telemetry.addData("Target Goal", "%s (%.1f, %.1f)",
                        currentAlliance, GOAL_X, targetGoalY);
                telemetry.addData("Distance", "%.1f in", distanceToGoal);

            } else {
                Pose2d absoluteCameraPose = camera.getAbsoluteRobotPose();
                if (absoluteCameraPose != null) {
                    mecaTank.setPoseEstimate(absoluteCameraPose);
                    telemetry.addData("Pose Status", "RE-CALIBRATED from AprilTag");
                } else {
                    telemetry.addData("Pose Status", "Coasting on Odometry (Full)");
                }

                mecaTank.setDrivePowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            }

            telemetry.addData("Current Pose (X, Y, H)", "%.2f, %.2f, %.2f",
                    currentPose.getX(),
                    currentPose.getY(),
                    Math.toDegrees(currentPose.getHeading()));
            telemetry.update();

            if(gamepad2.a){
                rollers.setPower(1.0);
                transferRollers.setPower(0);
            }else if(gamepad2.b){
                rollers.setPower(0);
            }
            if(gamepad2.y){
                resetOuttake();
                flywheelsActive = false;
            }else if(gamepad2.right_trigger > 0.1){
                shoot();
            }else if(gamepad1.dpad_up){
                hoodPos = hoodClosePos;
                flywheels.setCustomVelocityPID(shooterCloseVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad1.dpad_down){
                hoodPos = hoodFarPos;
                flywheels.setCustomVelocityPID(shooterFarVel, 0.008, 0.015, 0.0001, 0.000426);
            }else if(gamepad2.x){
                prepShooter(shooterFarVel);
                hoodPos = hoodFarPos;
                flywheelsActive = true;
            }else if(gamepad2.right_bumper){
                flywheels.setCustomVelocityPID(-800, 0.008, 0.015, 0.0001, 0.000426);
                transferRollers.setPower(-0.7);
                rollers.setPower(-0.9);
                flywheelsActive = false;
            }else if(gamepad2.left_bumper){
                flywheels.setCustomVelocityPID(0, 0.008, 0.015, 0.0001, 0.000426);
            }else if(hoodUp && !lastUp) {
                hoodPos -= 0.15;
            }else if (hoodDown && !lastDown) {
                hoodPos += 0.15;
            }

            lastUp = hoodUp;
            lastDown = hoodDown;
            hoodPos = Range.clip(hoodPos, 0, 1);


            hoodAdjuster.setPosition(hoodPos);
            telemetry.update();
        }
    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    private void prepShooter(double vel){
        flywheels.setCustomVelocityPID(vel, 0.008, 0.015, 0.0001, 0.000426);
        transferRollers.setPower(0);
    }
    private void shoot(){
        transferArtifacts();
        telemetry.update();
    }
    private void transferArtifacts(){
        rollers.setPower(1.0);
        transferRollers.setPower(0.8);
        telemetry.update();
    }
    private void resetOuttake(){
        rollers.setPower(0);
        transferRollers.setPower(0);
    }
}

