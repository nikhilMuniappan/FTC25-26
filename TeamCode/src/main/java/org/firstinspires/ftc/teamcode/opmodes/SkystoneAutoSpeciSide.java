package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@Disabled
@Autonomous
public class SkystoneAutoSpeciSide extends LinearOpMode{
    public DcMotor RightFront;
    public DcMotor LeftFront;
    public DcMotor RightBack;
    public DcMotor LeftBack;

    public Servo ClawServo;
    public CRServo RPServo;

    @Override
    public void runOpMode() throws InterruptedException{
        RightFront = hardwareMap.get(DcMotor.class, "RFM");
        LeftFront = hardwareMap.get(DcMotor.class, "LFM");
        RightBack = hardwareMap.get(DcMotor.class, "RBM");
        LeftBack = hardwareMap.get(DcMotor.class, "LBM");

        ClawServo = hardwareMap.get(Servo.class, "CLS");
        RPServo = hardwareMap.get(CRServo.class, "RPS");

        waitForStart();

        while(!isStopRequested()){
            ClawServo.setPosition(0);
            moveForward(0.5, 500);
            RightFront.setPower(0.5);
            RightBack.setPower(0.5);
            LeftFront.setPower(-0.5);
            LeftBack.setPower(-0.5);
            sleep(800);
            RightFront.setPower(0);
            LeftFront.setPower(0);
            RightBack.setPower(0);
            LeftBack.setPower(0);
            moveForward(0.4, 1200);

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
