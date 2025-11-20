package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Obelisk Motif Detection", group="Autonomous")
public class obeliskTesting extends LinearOpMode {

    DecodeCAM CAM = new DecodeCAM();


    @Override
    public void runOpMode() throws InterruptedException{
        CAM.init(hardwareMap.appContext, hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            String motif = CAM.getMotif();
            telemetry.addData("Motif", motif);
            telemetry.update();
            CAM.getGoalTagData();
        }

        CAM.stop();
    }

}
