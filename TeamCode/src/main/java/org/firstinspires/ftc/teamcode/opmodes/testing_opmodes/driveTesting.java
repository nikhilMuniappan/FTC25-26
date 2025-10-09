package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Config
@TeleOp
public class driveTesting extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    @Override
    public void runOpMode() throws InterruptedException{
        FrontLeft=hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight=hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft=hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight=hardwareMap.get(DcMotor.class, "BackRight");
        waitForStart();
        while(!isStopRequested()&& opModeIsActive()) {
            FrontLeft.setPower(gamepad1.left_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);
        }
    }
}
