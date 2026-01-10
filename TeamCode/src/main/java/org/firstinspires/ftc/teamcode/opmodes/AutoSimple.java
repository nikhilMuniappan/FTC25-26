package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
@Disabled
@Autonomous
public class AutoSimple extends LinearOpMode{
    private DcMotor RightFront;
    public DcMotor LeftFront;
    public DcMotor RightBack;
    public DcMotor LeftBack;

    private DcMotor rollers;
    private NGMotor flywheels;
    private DcMotor transferRollers;
    private Servo hoodAdjuster;

    @Override
    public void runOpMode() throws InterruptedException{
        RightFront = hardwareMap.get(DcMotor.class, "frontRight");
        LeftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        RightBack = hardwareMap.get(DcMotor.class, "backRight");
        LeftBack = hardwareMap.get(DcMotor.class, "backLeft");

        rollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.rollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
        hoodAdjuster = hardwareMap.get(Servo.class, DECODERobotConstants.hoodAdjuster);
        transferRollers = hardwareMap.get(DcMotor.class, DECODERobotConstants.transferRollers);

        flywheels.init();
        flywheels.setZeroPowerBehavior_Brake();

        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferRollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hoodAdjuster.setPosition(DECODERobotConstants.hoodStartPos);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        transferRollers.setDirection(DcMotor.Direction.REVERSE);

        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(!isStopRequested()){
            moveBack(0.45, 1450);
            rollers.setPower(0.25);
            flywheels.setVelocity(1200);
            hoodAdjuster.setPosition(DECODERobotConstants.farShootPos);
            sleep(3500);
            rollers.setPower(0.65);
            transferRollers.setPower(0.5);
            sleep(12000);
            flywheels.setVelocity(0);
            rollers.setPower(0);
            transferRollers.setPower(0);
            RightFront.setPower(-1);
            RightBack.setPower(1);
            LeftBack.setPower(-1);
            LeftFront.setPower(1);
            sleep(1000);
            RightFront.setPower(0);
            RightBack.setPower(0);
            LeftBack.setPower(0);
            LeftFront.setPower(0);
            break;
        }
    }
    private void moveForward(double speed, int time){
        RightFront.setPower(speed);
        LeftFront.setPower(speed);
        RightBack.setPower(speed);
        LeftBack.setPower(speed);
        sleep(time);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        LeftBack.setPower(0);
    }
    private void moveBack(double speed, int time){
        RightFront.setPower(-speed);
        LeftFront.setPower(-speed);
        RightBack.setPower(-speed);
        LeftBack.setPower(-speed);
        sleep(time);
        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightBack.setPower(0);
        LeftBack.setPower(0);
    }

}
