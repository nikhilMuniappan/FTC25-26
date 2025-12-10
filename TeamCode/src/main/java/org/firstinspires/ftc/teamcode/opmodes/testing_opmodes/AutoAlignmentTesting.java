package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecaTank;

@Config
@TeleOp
public class AutoAlignmentTesting extends LinearOpMode {
    MecaTank robot;

    @Override
    public void runOpMode() throws InterruptedException{
        robot = new MecaTank(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.right_bumper){
                robot.turnToAngle(55, this);
            }
            telemetry.update();
        }
    }

}
