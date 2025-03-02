package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.library.Potentiometer;

public class PotTesting extends TestingOpMode {
    Potentiometer pot;
    public static int position_at_0 = 180;
    public static int position_at_90 = 1380;
    public static double real_zero_degrees = 15;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        pot = new Potentiometer(hardwareMap, telemetry, "pot");

        // Wait for the Play button to be pressed

        waitForStart();
        while (opModeIsActive()) {
            pot.set0Position(position_at_0);
            pot.set90Position(position_at_90);
            pot.setRealZeroDegrees(real_zero_degrees);
            telemetry.addData("Pot: Arm Angle", pot.getAngle());
            telemetry.addData("Pot: Arm Position", pot.getCurrentPosition());
            telemetry.addData("Pot: Voltage", pot.getVoltage());
            telemetry.update();
        }

    }
}
