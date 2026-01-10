package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.library.NGMotor;

@Disabled
@Autonomous(name="Auto", group="Autonomous")
public class VinayakAuto extends LinearOpMode {

    NGMotor motor1, motor2, motor3, motor4;

    @Override
    public void runOpMode() {

        motor1 = new NGMotor(hardwareMap, telemetry, "motor1");
        motor2 = new NGMotor(hardwareMap, telemetry, "motor2");
        motor3 = new NGMotor(hardwareMap, telemetry, "motor3");
        motor4 = new NGMotor(hardwareMap, telemetry, "motor4");

        // Initialize motors
        motor1.init();
        motor2.init();
        motor3.init();
        motor4.init();

        // Reverse right side
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Ready – 30s Red Left Power Auto");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        long start = System.currentTimeMillis();

        // START OF 30-SECOND ROUTINE

        // 1. Drive forward fast (3.0 sec)
        drivePower(0.4);
        sleep(1500);

        // 2. Slow wiggle turn right (1.5 sec)
        turnRightPower(0.25);
        sleep(1500);

        // 3. Drive forward slow toward backdrop (2.2 sec)
        drivePower(0.45);
        sleep(2200);

        // 4. Replace stop with slow forward shuffle (1.5 sec)
        drivePower(0.3);
        sleep(1500);

        // 5. Back up (2.5 sec)
        driveBackwardPower(0.6);
        sleep(2500);

        // 6. Turn left 45° (1.4 sec)
        turnLeftPower(0.55);
        sleep(1400);

        // 7. Long forward sweep (3.3 sec)
        drivePower(0.5);
        sleep(3300);

        // 8. Long backward sweep (3.3 sec)
        driveBackwardPower(0.5);
        sleep(3300);

        // 9. Larger left-right wiggle (2.5 sec total)
        turnLeftPower(0.45);
        sleep(1300);

        turnRightPower(0.45);
        sleep(1200);

        // 10. Long slow roll into parking (3.5 sec)
        drivePower(0.35);
        sleep(3500);

        // Movement time ≈ 24.7s (target: 25)
        // Remaining time → wait until exactly 30s total

        stopAll();
        long elapsed = System.currentTimeMillis() - start;
        if (elapsed < 30000)
            sleep(30000 - elapsed);

        stopAll();
    }


    // POWER HELPER FUNCTIONS

    private void drivePower(double p) {
        motor1.setPower(p);
        motor2.setPower(p);
        motor3.setPower(p);
        motor4.setPower(p);
    }

    private void driveBackwardPower(double p) {
        motor1.setPower(-p);
        motor2.setPower(-p);
        motor3.setPower(-p);
        motor4.setPower(-p);
    }

    private void turnRightPower(double p) {
        motor1.setPower(p);
        motor2.setPower(p);
        motor3.setPower(-p);
        motor4.setPower(-p);
    }

    private void turnLeftPower(double p) {
        motor1.setPower(-p);
        motor2.setPower(-p);
        motor3.setPower(p);
        motor4.setPower(p);
    }

    private void stopAll() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
