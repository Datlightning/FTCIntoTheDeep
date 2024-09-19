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
    public static double motor_power = 0;
    public static int target_position = 0;
    public static boolean controller =false;
    public static double target_angle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(motor_power == 0){
                intake.update();
            }else{
                intake.setPower(motor_power);
            }
            intake.moveArm(target_position);
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
