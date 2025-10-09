package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DECODERobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.Potentiometer;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.NGAutoOpMode;
@Config
public class Intake2_0 {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;
    public NGMotor rollers;
    public NGMotor flywheels;

    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        timer = new ElapsedTime();
        rollers = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.rollers);
        flywheels = new NGMotor(hardwareMap, telemetry, DECODERobotConstants.flywheels);
    }
    public Intake2_0(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer) {
        this(hardwareMap, telemetry);
        this.timer = timer;
    }

    public void runRollers(double power){
        rollers.setPower(power);
    }
    public void stopRollers(){
        rollers.setPower(0);
    }
    public void runFlywheels(double power){
        flywheels.setPower(power);
    }
    public void stopFlywheels(){
        flywheels.setPower(0);
    }
    public Action shoot(double time, double p){
        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> runFlywheels(1)),
                        new SleepAction(time),
                        new InstantAction(() -> stopFlywheels())
                )
        );
    }
    public Action collect(double time) {
        return new ParallelAction(
                new SequentialAction(
                        new InstantAction(() -> runRollers(1)),
                        new SleepAction(time),
                        new InstantAction(() -> stopRollers())
                )
        );
    }
}
