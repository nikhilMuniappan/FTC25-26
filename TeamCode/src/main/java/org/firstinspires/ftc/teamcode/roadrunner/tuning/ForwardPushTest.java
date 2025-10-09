package org.firstinspires.ftc.teamcode.roadrunner.tuning;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class ForwardPushTest extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    @Override
    public void runOpMode() throws InterruptedException{

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();

        while(opModeIsActive()&& !isStopRequested()){
            frontRight.setPower(-gamepad1.right_stick_y);
            backRight.setPower(-gamepad1.right_stick_y);
            frontLeft.setPower(gamepad1.left_stick_y);
            backLeft.setPower(gamepad1.left_stick_y);
            int ticksAVG = ((frontRight.getCurrentPosition()+frontLeft.getCurrentPosition()+backRight.getCurrentPosition()+backLeft.getCurrentPosition())/4);

        telemetry.addData("frontRight Ticks: ", frontRight.getCurrentPosition());
        telemetry.addData("frontLeft Ticks: ", frontLeft.getCurrentPosition());
        telemetry.addData("backRight Ticks: ", backRight.getCurrentPosition());
        telemetry.addData("backLeft Ticks: ", backLeft.getCurrentPosition());
        telemetry.addData("Average Ticks: ", ticksAVG);
        telemetry.update();
        }
    }
}
