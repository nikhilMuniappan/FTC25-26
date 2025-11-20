package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DECODERobotConstants;
@Config
@TeleOp
public class servoTesting extends LinearOpMode{
    private CRServo hoodAdjuster;

    @Override
    public void runOpMode() throws InterruptedException{
        hoodAdjuster = hardwareMap.get(CRServo.class, DECODERobotConstants.hoodAdjuster);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            if(gamepad1.a){
                hoodAdjuster.setPower(1);
            }else if(gamepad1.b){
                hoodAdjuster.setPower(0);
            }
        }
    }
}
