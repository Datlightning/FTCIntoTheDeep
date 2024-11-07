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
    public static double claw_position = 0.85;
    public static double wrist_position = 1;
    public static double rotation_power = 0;
    public static double slide_power = 0;
    public static int target_position = 0;
    public static int slide_position = 0;
    public static boolean controller =false;
    public static double target_angle = 0;
    public static boolean testing_level = false;
    public static boolean delay = false;
    public static boolean disable_pid_movement = false;
    public static boolean use_motion_profile_arm = false;
    public static boolean use_offset_calibrate = false;
    public static boolean use_motion_profile_slide = false;
    public static int slide_hard_stop = 1400;
    public static double claw_height = 0.75;
    public static int increment = 30;
    public static double VEL = 3900;
    public static double ACCEL = 3000;
    public static double SLIDE_VEL = 10000;
    public static double SLIDE_ACCEL = 9000;

    public static double DISTANCE_FILTER = 0.6;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {

            intake.update();
            intake.distance.setFilter(DISTANCE_FILTER);
            intake.setRotationPower(rotation_power == 0 ? gamepad1.left_stick_y : rotation_power);
            intake.setSlidePower(slide_power == 0 ? gamepad2.left_stick_y : slide_power);

            intake.slides.setMax(slide_hard_stop);
            intake.slides.setUseMotionProfile(use_motion_profile_slide);
            intake.arm.setUseMotionProfile(use_motion_profile_arm);

            if(controller){
                intake.setFourBar(true);
                intake.setTargetAngle(target_angle);
            }else{
                intake.moveWrist(wrist_position);
            }
            if(use_offset_calibrate){
                intake.calculateOffset();
                use_offset_calibrate = false;
            }
            intake.moveClaw(claw_position);

            intake.setTargetHeight(claw_height);
            intake.calculateArmPosition(intake.slides.getCurrentPosition());

            intake.arm.setMaxVelocity(VEL);
            intake.arm.setMaxAcceleration(ACCEL);

            intake.slides.setMaxVelocity(SLIDE_VEL);
            intake.slides.setMaxAcceleration(SLIDE_ACCEL);

            if(testing_level){
                intake.moveArm(intake.calculateArmPosition(intake.slides.getCurrentPosition()));
            }
            if(gamepad1.left_bumper){
                intake.moveSlides(intake.slides.targetPos + increment);
            }else if(gamepad1.right_bumper){
                intake.moveSlides(intake.slides.targetPos - increment);
            }
            if(testing_level){
                intake.moveArm(intake.calculateArmPosition(intake.slides.getCurrentPosition()));
            }else{
                if(!disable_pid_movement) {
                    intake.moveArm(target_position);
                    intake.moveSlides(slide_position);
                }
            }
            if(gamepad1.left_bumper){
                if(delay){
                    intake.moveArmWithDelay(increment);
                }else {
                    intake.moveSlides(intake.slides.targetPos + increment);
                }
            }else if(gamepad1.right_bumper){
                if(delay){
                    intake.moveSlidesWithDelay(-increment);
                }else {
                    intake.moveSlides(intake.slides.targetPos - increment);
                }
            }
            intake.telemetry();
            telemetry.update();
        }

    }
}
