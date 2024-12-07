package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Control;
import org.firstinspires.ftc.teamcode.library.Subsystem;

import java.util.concurrent.TimeUnit;

@Config
public class MecaTank extends Subsystem {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private RobotConstants config;

    public  TrafficLight trafficLight;
    private LazyImu imu; // IMU for field-centric control

    public Distance distance;
    public Distance rear_distance;
    private Telemetry telemetry;
    private double MAX_DRIVE_SPEED = 1;
    private boolean left_strafe = false;
    private boolean right_strafe = false;
    private boolean force_exit = false;

    private boolean fast_drive_direction = false;

    public double inPerTick = 1/1836.3936507936507936507936507937;

    private boolean use_dead_wheel = false;
    private double distance_to_target = 0;
    private double starting_motion_profile_time = 0;

    private double fast_drive_speed = 0;
    private double starting_pos = 0;
    private double time_stop = 0;
    double currentFilterEstimate = 0;
    double previousFilterEstimate = 0;
    public static double kP = 0.02;  // Proportional constant
    public static double kI = 0.001;  // Integral constant
    public static double kD = 0.0005;

    public static double kP_heading = 0.04;
    public static double kD_heading = 0.002;
    public static boolean  motion_profile = true;
    public static double MAX_VEL = 5;
    public static double MAX_ACCEL = 0.8;
    public static double MAX_DECEL = -0.2;
    public static double kF = -0.05;// Feedforward constant
    double previousError = 0;
    double integral = 0;
    private boolean fast_drive = false;
    private double target = 0;

    private boolean update_distance = true;

    private boolean front_distance = true;
    public static double a = 0.6;
    private boolean auto_move = false;
    private double previousHeadingError = 0;
    double error;

    double targetHeading = 0;

    private ElapsedTime timer;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance);
        imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);

        timer = new ElapsedTime();
        this.telemetry = telemetry;

    }
    public boolean isFrontDistance(){
        return front_distance;
    }

    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, TrafficLight trafficLight){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        this.trafficLight = trafficLight;
        imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        this.timer = timer;
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

    }
    public void setDistanceType(boolean front){
        this.front_distance = front;
    }
    public void mountFrontDistance(Distance distance){
        update_distance = false;
        this.distance = distance;
    }
    public void mountRearDistance(Distance distance){
        update_distance = false;
        this.rear_distance = distance;
    }
    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    private double getHeading() {
        return imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }
    private double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
        double xRotated = x * cosA - y * sinA;
        double yRotated = x * sinA + y * cosA;
        return new double[] {xRotated, yRotated};
    }
   public void forceExit(){
        force_exit = true;
   }
   void set_individual_powers(double fl_power, double fr_power, double bl_power, double br_power){
        frontRight.setPower(fr_power);
        frontLeft.setPower(fl_power);
        backLeft.setPower(bl_power);
        backRight.setPower(br_power);
    }
    public void setUseDeadwheel(boolean on){
        use_dead_wheel = on;
    }
    public double getDistance(){

        if(front_distance){
            return distance.getFilteredDist();
        }
        return rear_distance.getFilteredDist();
    }
    public double getLinearDeadwheel(){
        return (backLeft.getCurrentPosition() * PARAMS.inPerTick);


    }
    public void clearLinearDeadwheel(){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPowers(double left_stick_x, double left_stick_y, double right_stick_x) {
        double heading = getHeading();  // Get robot heading

        // Rotate joystick inputs for field-centric control
        double[] rotated = rotateVector(left_stick_x, left_stick_y, -heading);
        double x = rotated[0];
        double y = rotated[1];

        // Mecanum drive equations
        double frontLeftPower = y + x + right_stick_x;
        double frontRightPower = y - x - right_stick_x;
        double backLeftPower = y - x + right_stick_x;
        double backRightPower = y + x - right_stick_x;

        // Normalize powers so that no value exceeds 1
        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Apply the calculated power values to the motors
        frontLeft.setPower(frontLeftPower * MAX_DRIVE_SPEED);
        frontRight.setPower(frontRightPower * MAX_DRIVE_SPEED);
        backLeft.setPower(backLeftPower * MAX_DRIVE_SPEED);
        backRight.setPower(backRightPower * MAX_DRIVE_SPEED);
    }

    public void setPowers(double left_stick_y, double right_stick_y, double left_trigger, double right_trigger){
        if(auto_move){
            return;
//            if(left_stick_y == 0 && right_stick_y == 0 && left_trigger == 0 && right_trigger == 0){
//                return;
//            }
        }
        left_strafe = left_trigger != 0;
        right_strafe = right_trigger != 0;
        if (left_strafe) {
            double posPower = sameSignSqrt(left_trigger);
            double negPower = sameSignSqrt(-left_trigger);
            frontLeft.setPower(negPower * MAX_DRIVE_SPEED);
            backRight.setPower(negPower * MAX_DRIVE_SPEED);
            frontRight.setPower(posPower * MAX_DRIVE_SPEED);
            backLeft.setPower(posPower * MAX_DRIVE_SPEED);
            return;
        }
        if (right_strafe) {
            double posPower = sameSignSqrt(right_trigger);
            double negPower = sameSignSqrt(-right_trigger);
            frontLeft.setPower(posPower * MAX_DRIVE_SPEED);
            backRight.setPower(posPower * MAX_DRIVE_SPEED);
            frontRight.setPower(negPower * MAX_DRIVE_SPEED);
            backLeft.setPower(negPower * MAX_DRIVE_SPEED);
            return;
        }

        double leftPower = sameSignSqrt(-left_stick_y);
        double rightPower = sameSignSqrt(-right_stick_y);
        frontLeft.setPower(leftPower * MAX_DRIVE_SPEED);
        backLeft.setPower(leftPower * MAX_DRIVE_SPEED);
        frontRight.setPower(rightPower * MAX_DRIVE_SPEED);
        backRight.setPower(rightPower * MAX_DRIVE_SPEED);
    }
    public void LivePIDToDistance(double distance){
        if (target != distance){
            previousError = 0;
            integral = 0;
            distance_to_target = distance - getDistance();
            starting_motion_profile_time = timer.time();
            starting_pos = getDistance();
            if (use_dead_wheel && !front_distance){
                distance_to_target *= -1;
            }
            clearLinearDeadwheel();
            motion_profile = !(Math.abs(distance_to_target) < 2);
        }
        fast_drive = false;
        auto_move = true;
        imu.get().resetYaw();
        targetHeading = getHeading();
        target = distance;
    }
    public void PIDToDistance(double distance){
        if (target != distance || !auto_move){
            previousError = 0;
            integral = 0;
            distance_to_target = distance - getDistance();
            starting_motion_profile_time = timer.seconds();
            starting_pos = getDistance();
            auto_move = true;
            fast_drive = false;
            if (use_dead_wheel && !front_distance){
                distance_to_target *= -1;
            }
            clearLinearDeadwheel();
            motion_profile = !(Math.abs(distance_to_target) < 2);
            imu.get().resetYaw();
            targetHeading = getHeading();
            target = distance;
        }
    }
    public void DrivePastDistance(double distance, double speed){
        if(target != distance) {
            fast_drive = true;
            auto_move = true;
            target = distance;
            fast_drive_direction = front_distance ? (getDistance() > distance) : (getDistance() < distance);
            fast_drive_speed = fast_drive_direction ? speed : -speed;

        }

    }



    @Override
    public void update() {
        if(update_distance) {
            distance.update();
            rear_distance.update();
        }
        trafficLight.update();


        if(force_exit){
            auto_move = false;
            force_exit = false;
            frontLeft.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
        }
        // Declare variables outside of the loop for PID

        if (auto_move) {
            double currentHeading = getHeading();// Get current heading (yaw)
            telemetry.addData("Auto Drive Direction", fast_drive_direction);
            telemetry.addData("Auto Drive Front Distance", front_distance);
            telemetry.addData("Auto Drive Reading Distance Value", getDistance());
            telemetry.addData("Auto Drive Target", target);
            telemetry.addData("Auto Drive Speed", fast_drive_speed);
            telemetry.addData("Auto Drive Target Heading", targetHeading);
            telemetry.addData("Auto Drive Current Heading", currentHeading);
            if (fast_drive) {



                boolean completed = fast_drive_direction ? (front_distance ? (getDistance() < target) : (getDistance() > target)) : (front_distance ? (getDistance() > target) : (getDistance() < target));

                if (completed) {
                    fast_drive = false;
                    auto_move = false;
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);
                } else {
                    trafficLight.red(true);
                    frontLeft.setPower(fast_drive_speed );
                    frontRight.setPower(fast_drive_speed );
                    backLeft.setPower(fast_drive_speed );
                    backRight.setPower(fast_drive_speed );
                }

            } else {
                if(use_dead_wheel) {
                    if(motion_profile){
                        error = Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) - getLinearDeadwheel();
                    }else{
                        error = distance_to_target - getLinearDeadwheel();
                    }
                }else {
                    if (motion_profile) {
                        error = (Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) + starting_pos) - getDistance();
                    } else {
                        error = target - getDistance();
                    }
                    error *= front_distance ? -1 : 1;
                }
                telemetry.addData("Auto Drive Error", error);

                double time_passed = timer.seconds() - time_stop;

                if (Math.abs(getDistance() - target) < 0.5) {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    auto_move = false;
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);

                    previousError = 0;  // Reset for future runs
                    integral = 0;       // Reset for future runs
                } else {
                    trafficLight.red(true);

                    double errorChange = (error - previousError);
                    currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
                    previousFilterEstimate = currentFilterEstimate;

                    integral += error * time_passed;
                    double derivative = currentFilterEstimate / time_passed;
                    double power = kP * error + kI * integral + kD * derivative;

                    power = Math.max(-0.5, Math.min(0.5, power));
                    power -= kF * Math.signum(error);

                    // Calculate heading correction
                    double headingError = currentHeading - targetHeading;
                    double headingCorrection = kP_heading * headingError +
                            kD_heading * (headingError - previousHeadingError) / time_passed;
                    previousHeadingError = headingError;

                    // Adjust motor powers for heading correction
                    frontLeft.setPower(power + headingCorrection);
                    backLeft.setPower(power + headingCorrection);
                    backRight.setPower(power - headingCorrection);
                    frontRight.setPower(power - headingCorrection);

                    previousError = error;
                    time_stop = timer.seconds();
                }
            }
        }

    }
    public boolean isBusy(){
        return auto_move;
    }
    @Override
    public void telemetry() {
        trafficLight.telemetry();;
//        distance.telemetry();
//        rear_distance.telemetry();
        telemetry.addData("Robot Heading", getHeading());
        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());
        telemetry.addData("MecaTank Perceived Distance", getDistance());

    }

    @Override
    public void init() {

    }

    public void setMaxPower(double v) {
        MAX_DRIVE_SPEED = v;
    }
}