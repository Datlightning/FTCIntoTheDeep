package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.StrafeTest;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

import java.lang.annotation.Target;

@TeleOp
@Config
public class IntakeTesting extends TestingOpMode {
    Intake intake;
    TrafficLight trafficLight;
    public static double claw_position = RobotConstants.claw_closed;
    public static double wrist_angle = 0;
    public static double claw_angle = 0;
    public static double rotation_power = 0;
    public static double slide_power = 0;
    public static int arm_position = 0;
    public static int slide_position = 0;

    private int past_arm_position = 0;
    private int past_slide_position = 0;
    public static boolean controller = false;
    public static double target_angle = 0;
    public static boolean testing_level = false;
    public static boolean use_motion_profile_arm = true;
    public static boolean use_offset_calibrate = false;
    public static boolean use_motion_profile_slide = true;

    public static boolean enable_arm_motor = true;

    public static boolean extend_slide_with_distance = false;
    public static int slide_hard_stop = 1400;
    public static double claw_height = 0.75;
    public static int increment = 30;
    public static double VEL = 5000;
    public static double ACCEL = 5000;
    public static double SLIDE_VEL = 12000;
    public static double SLIDE_ACCEL = 9000;

    public static double SLIDE_DECEL = 3000;

    public static double DECEL = 1500;
    MecaTank mecaTank;
    public static boolean drive_pid_on = false;
    public static boolean front_distance = true;
    public static boolean fast_pid = false;
    public static double distance = 8;
    public static double DISTANCE_FILTER = 0.6;

    public static double drive_speed = 0.15;

    public static boolean distance_scope = false;
    private boolean past_distance_scope = false;

    public static boolean lower_arm_until_0_velo = false;
    public static double lower_arm_speed = -0.4;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        intake.init();
        mecaTank = new MecaTank(hardwareMap, telemetry, timer, trafficLight);

        mecaTank.init();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if(distance_scope != past_distance_scope){
                if(!distance_scope){
                    trafficLight.green(false);
                }
            }
            past_distance_scope = distance_scope;


            mecaTank.setDistanceType(front_distance);
            if(arm_position != past_arm_position){
                intake.moveArm(arm_position);

            }
            if(lower_arm_until_0_velo){
                lower_arm_until_0_velo = false;
                intake.moveArmUntilZeroSpeed(lower_arm_speed);
            }
            if(slide_position != past_slide_position){
                intake.moveSlides(slide_position);
            }
            past_arm_position = arm_position;
            past_slide_position = slide_position;




            intake.distance.setFilter(DISTANCE_FILTER);

            intake.slides.setMax(slide_hard_stop);
            intake.slides.setUseMotionProfile(use_motion_profile_slide);
            intake.arm.setUseMotionProfile(use_motion_profile_arm);

            if(controller){
                intake.setFourBar(true);
                intake.setTargetAngle(target_angle);
                intake.turnClaw(claw_angle);
            }else{
                intake.turnAndRotateClaw(wrist_angle, claw_angle);
            }
            if(use_offset_calibrate){
                intake.calculateOffset();
                use_offset_calibrate = false;
            }
                intake.arm.enableMotor(enable_arm_motor);

            intake.moveClaw(claw_position);

            intake.setTargetHeight(claw_height);
            intake.calculateArmPosition(intake.slides.getCurrentPosition());

            intake.arm.setMaxVelocity(VEL);
            intake.arm.setMaxAcceleration(ACCEL);
            intake.arm.setMaxDeceleration(DECEL);

            intake.slides.setMaxVelocity(SLIDE_VEL);
            intake.slides.setMaxAcceleration(SLIDE_ACCEL);
            intake.slides.setMaxDeceleration(SLIDE_DECEL);


            intake.slides.setManualPower(gamepad2.left_stick_y);
            intake.arm.setManualPower(gamepad2.right_stick_y);
            if(testing_level){
                intake.moveArm(intake.calculateArmPosition(intake.slides.getCurrentPosition()));
            }
            if(extend_slide_with_distance){
                intake.moveSlides(intake.calculateSlidePositionForFloorPickup(intake.distance.getFilteredDist()));
            }

            if(gamepad1.left_bumper){
                    intake.moveSlides(intake.slides.targetPos + increment);
                    slide_position += increment;
            }else if(gamepad1.right_bumper){
                    intake.moveSlides(intake.slides.targetPos - increment);
                    slide_position -= increment;
            }

            if(!mecaTank.isBusy()){
                mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            }
            if(!drive_pid_on) {
                mecaTank.forceExit();
            }else{
                if(fast_pid){
                    mecaTank.DrivePastDistance(distance, drive_speed);
                }else {
                    mecaTank.PIDToDistance(distance);
                }
            }
            if(distance_scope) {
                trafficLight.green(intake.distance.getFilteredDist() < 12);
            }
            intake.distance.setOn(true);
            mecaTank.rear_distance.setOn(true);
            telemetry.addData("Target Distance", distance);
            mecaTank.telemetry();
            mecaTank.update();
            intake.update();
            intake.distance.update();
            intake.distance.telemetry();
            mecaTank.rear_distance.update();
            mecaTank.rear_distance.telemetry();
            telemetry.addData("Slide Predicted Distance", intake.calculateSlidePositionForFloorPickup(intake.distance.getFilteredDist()));
            intake.telemetry();
            telemetry.update();
        }

    }
}
