package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
@Config
public class IntakeTesting extends LinearOpMode {
    Intake intake;
    public static double claw_position = 0.3;
    public static double wrist_position = 0.3;
    public static double rotation_power = 0;
    public static double slide_power = 0;
    public static int target_position = 0;
    public static int slide_position = 0;
    public static boolean controller =false;
    public static double target_angle = 0;
    public static boolean enable_pid = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(enable_pid) {
                intake.update();
            }else {

                intake.setRotationPower(rotation_power == 0 ? gamepad1.left_stick_y : rotation_power);
                intake.setSlidePower(slide_power == 0 ? gamepad2.left_stick_y : slide_power);
            }
            intake.moveArm(target_position);
            intake.moveSlides(slide_position);
            if(controller){
                intake.setFourBar(true);
                intake.setTargetAngle(target_angle);
            }else{
                intake.moveWrist(wrist_position);
            }
            intake.moveClaw(claw_position);




            intake.telemetry();
            telemetry.update();
        }

    }
}
