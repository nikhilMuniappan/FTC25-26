package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.MultiClick;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;


@Config
@TeleOp
public class ATeleOpNikhil extends LinearOpMode{
    private NGMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rollers;
    private DcMotor flywheels;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.fl);
        frontRight = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.fr);
        backLeft = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.bl);
        backRight = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.br);
        rollers = hardwareMap.get(DcMotor.class, "rollers");
        flywheels = hardwareMap.get(DcMotor.class, "flywheels");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_y==0) {
                frontLeft.setDrivePower(gamepad1.left_stick_y);
                backLeft.setDrivePower(gamepad1.left_stick_y);
            }else if(gamepad1.right_stick_y != 0 && gamepad1.left_stick_y==0){
                frontRight.setDrivePower(gamepad1.right_stick_y);
                backRight.setDrivePower(gamepad1.right_stick_y);
            }else if(gamepad1.right_trigger!=0){
                strafeRight(1, gamepad1.right_trigger);
            }else if(gamepad1.left_trigger!=0){
                strafeLeft(1, gamepad1.left_trigger);
            }else if(gamepad1.left_stick_y==0 && gamepad1.right_stick_y==0){
                frontLeft.setDrivePower(0);
                frontRight.setDrivePower(0);
                backLeft.setDrivePower(0);
                backRight.setDrivePower(0);
            }else if(gamepad1.left_stick_y != 0 && gamepad1.right_stick_y != 0){
                frontRight.setDrivePower(gamepad1.right_stick_y);
                backRight.setDrivePower(gamepad1.right_stick_y);
                frontLeft.setDrivePower(gamepad1.left_stick_y);
                backLeft.setDrivePower(gamepad1.left_stick_y);
            }
            if(gamepad2.a){
                rollers.setPower(1.0);
            }else if(gamepad2.b){
                rollers.setPower(0);
            }
            flywheels.setPower(-gamepad2.right_trigger);
        }
    }
    private void strafeLeft(double L_STRAFE_POWER, double left_trigger){
        double posPower = sameSignSqrt(left_trigger);
        double negPower = sameSignSqrt(-left_trigger);
        frontRight.setDrivePower(L_STRAFE_POWER*negPower);
        backLeft.setDrivePower(L_STRAFE_POWER*negPower);
        frontLeft.setDrivePower(L_STRAFE_POWER*posPower);
        backRight.setDrivePower(L_STRAFE_POWER*posPower);
    }
    private void strafeRight(double R_STRAFE_POWER, double right_trigger){
        double posPower = sameSignSqrt(right_trigger);
        double negPower = sameSignSqrt(-right_trigger);
        frontRight.setDrivePower(R_STRAFE_POWER*posPower);
        backLeft.setDrivePower(R_STRAFE_POWER*posPower);
        frontLeft.setDrivePower(R_STRAFE_POWER*negPower);
        backRight.setDrivePower(R_STRAFE_POWER*negPower);
    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
}
