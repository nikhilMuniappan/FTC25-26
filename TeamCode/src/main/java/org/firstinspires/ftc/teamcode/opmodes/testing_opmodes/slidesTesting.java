package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class slidesTesting extends LinearOpMode{

    private DcMotor slides;
    @Override
    public void runOpMode() throws InterruptedException{
        slides=hardwareMap.get(DcMotor.class, "slides");

        waitForStart();
        while(!isStopRequested()&& opModeIsActive()) {

            if(gamepad1.right_bumper){
                slides.setPower(0.4);
                sleep(50);
                slides.setPower(0);
            }
            if(gamepad1.left_bumper){
                slides.setPower(-0.4);
                sleep(50);
                slides.setPower(0);
            }
        }
    }
    private void moveSlides(double power, int sec){
        slides.setPower(power);
        sleep(sec*1000);
        slides.setPower(0);
    }
    private void stopSlides(){
        slides.setPower(0);
    }
}

