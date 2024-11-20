package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;

@Config
@TeleOp
//@Disabled
public class DriveTesting extends LinearOpMode {
    MecaTank mecaTank;
    public static boolean pid_on = false;
    public static double distance = 8;
    public static boolean fast_drive = false;
    public static double speed = 0.5;
    public static boolean front_distance = true;
    public static boolean field_centric = false;
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecaTank = new MecaTank(hardwareMap, telemetry);

        mecaTank.init();
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            mecaTank.setDistanceType(front_distance);
            if(!pid_on) {
                mecaTank.forceExit();
            }else{
                if(fast_drive){
                    mecaTank.DrivePastDistance(distance, speed);
                }else {
                    mecaTank.PIDToDistance(distance);
                }
            }
            telemetry.addData("Target Distance", distance);
            mecaTank.telemetry();
            mecaTank.update();
            telemetry.update();
        }
    }
}
