package org.firstinspires.ftc.teamcode.testing;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DecodeCAM;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
@Config
@TeleOp
public class autoTurretTesting extends LinearOpMode {
    DecodeCAM CAM = new DecodeCAM();

    @Override
    public void runOpMode() throws InterruptedException{
        CAM.init(hardwareMap.appContext, hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CAM.getGoalTagData();
            if()
        }

        CAM.stop();
    }

}
