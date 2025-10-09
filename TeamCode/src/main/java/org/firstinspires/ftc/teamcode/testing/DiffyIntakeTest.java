package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class DiffyIntakeTest extends LinearOpMode {
    private Servo leftServo, rightServo, clawServo;
    private final Gamepad previous = new Gamepad();
    private final Gamepad current = new Gamepad();

    public static double tiltIncrement = 0.000000000000000000000005;
    public static double yawIncrement = 0.00000000000000000000000000000000005;
    public static double tiltPos = 0.33;
    public static double yawPos = 0.33;
    public static double clawOpenPos = 1.0;
    public static double clawClosedPos = 0.0;

    private boolean clawOpen = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        //Start: Claw closed, diffy half-way(midpoint)
        clawServo.setPosition(clawClosedPos);

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            boolean dpadPressed = false;
            double targetTilt = tiltPos;
            double targetYaw = yawPos;

            if (current.dpad_up) {
                targetTilt = clamp(tiltPos + tiltIncrement, 0.3, 0.7);
                dpadPressed = true;
            }
            if (current.dpad_down) {
                targetTilt = clamp(tiltPos - tiltIncrement, 0.3, 0.7);
                dpadPressed = true;
            }
            if (current.dpad_right) {
                targetYaw = clamp(yawPos + yawIncrement, 0.0, 0.08);
                dpadPressed = true;
            }
            if (current.dpad_left) {
                targetYaw = clamp(yawPos - yawIncrement, 0.0, 0.08);
                dpadPressed = true;
            }

            // Update positions only if D-pad pressed
            tiltPos = (current.dpad_up || current.dpad_down) ? targetTilt : tiltPos;
            yawPos = (current.dpad_right || current.dpad_left) ? targetYaw : yawPos;

            double leftPos = clamp(tiltPos + yawPos - 0.5, 0.3, 0.7);
            double rightPos = clamp(tiltPos - yawPos + 0.5, 0.3, 0.7);

            // Freeze servos at current position if D-pad not pressed
            if (!dpadPressed) {
                leftServo.setPosition(leftServo.getPosition());
                rightServo.setPosition(rightServo.getPosition());
            } else {
                leftServo.setPosition(leftPos);
                rightServo.setPosition(rightPos);
            }

            // a to open, a to close
            if (current.a && !lastA) {
                clawOpen = !clawOpen;
                clawServo.setPosition(clawOpen ? clawOpenPos : clawClosedPos);
            }
            lastA = current.a;

            telemetry.addData("Tilt", "%.2f", tiltPos);
            telemetry.addData("Yaw", "%.2f", yawPos);
            telemetry.addData("LeftServo", "%.2f", leftPos);
            telemetry.addData("RightServo", "%.2f", rightPos);
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();

            idle();
        }
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}


