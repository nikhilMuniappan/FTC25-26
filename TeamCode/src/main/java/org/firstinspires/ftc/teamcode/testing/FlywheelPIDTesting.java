package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.MultiClick;
import org.firstinspires.ftc.teamcode.library.NGMotor;
@Config
@TeleOp
public class FlywheelPIDTesting extends LinearOpMode{
    private NGMotor flywheels;
    private Servo hoodAdjuster;
    private DcMotor rollers;
    private DcMotor transferRollers;

    private final double FLYWHEEL_P = 0.08;
    private final double FLYWHEEL_I = 0.01;
    private final double FLYWHEEL_D = 0.01;
    private final double FLYWHEEL_F = 0.6;

    @Override
    public void runOpMode() throws InterruptedException{
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
        rollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.rollers);
        transferRollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.transferRollers);

        hoodAdjuster.setPosition(1.0);
        double pos = hoodAdjuster.getPosition();
        boolean lastUp = false;
        boolean lastDown = false;
        double targetVel = DECODERobotConstants.closeZoneShootingVel;
        flywheels.init();
        flywheels.setCustomVelocityPID(0.0, FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("hoodPos: ", hoodAdjuster.getPosition());
        telemetry.update();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            flywheels.updateFlywheels();

            if(gamepad2.dpad_right){
                flywheels.setCustomVelocityPID(1400, 0.0006, 0.0009, 0.000085, 0.000471);
            }else if(gamepad2.dpad_left){
                flywheels.setCustomVelocityPID(0, 0.0006, 0.0009, 0.000085, 0.000471);
            }
            if(gamepad2.a){
                rollers.setPower(1.0);
            }else if(gamepad2.b){
                rollers.setPower(0);
            }
            if(gamepad2.right_trigger>=0.1){
                transferRollers.setPower(1.0);
                rollers.setPower(1);
            }else if(gamepad2.y){
                rollers.setPower(0);
                transferRollers.setPower(0);
            }

            boolean up = gamepad2.dpad_up;
            boolean down = gamepad2.dpad_down;

                // detect new press
                if (up && !lastUp) {
                    pos -= 0.15;
                } else if (down && !lastDown) {
                    pos += 0.15;
                }

                // remember current state
                lastUp = up;
                lastDown = down;

                // clip within [0,1]
                pos = Range.clip(pos, 0, 1);
                hoodAdjuster.setPosition(pos);

            telemetry.addData("hoodPos: ", hoodAdjuster.getPosition());
            telemetry.addData("flywheel vel: ", flywheels.getVelocity());
            telemetry.update();
        }
    }
}
