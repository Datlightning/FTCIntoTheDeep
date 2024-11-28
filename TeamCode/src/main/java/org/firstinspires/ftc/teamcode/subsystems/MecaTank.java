package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    private boolean left_lock = false;
    private boolean right_lock = false;
    private boolean force_exit = false;

    private boolean fast_drive_direction = false;

    private double distance_to_target = 0;
    private double starting_motion_profile_time = 0;

    private double fast_drive_speed = 0;
    private double starting_pos = 0;
    private double time_stop = 0;
    double currentFilterEstimate = 0;
    double previousFilterEstimate = 0;
    public static double kP = 0.03;  // Proportional constant
    public static double kI = 0.01;  // Integral constant
    public static double kD = 0.0005;

    public static boolean  motion_profile = false;
    public static double MAX_VEL = 5;
    public static double MAX_ACCEL = 2;
    public static double MAX_DECEL = -0.5;
    public static double kF = -0.05;// Feedforward constant
    double previousError = 0;
    double integral = 0;
    private boolean fast_drive = false;
    private double target = 0;

    private boolean update_distance = true;

    private boolean front_distance = true;
    public static double a = 0.6;
    private boolean auto_move = false;
    double error;

    private ElapsedTime timer;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance);
        if(RobotConstants.imu_init) {
            this.imu = RobotConstants.imu;
        }
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
        if(RobotConstants.imu_init) {
            this.imu = RobotConstants.imu;
        }
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
    public double getDistance(){
        if(front_distance){
            return distance.getFilteredDist();
        }
        return rear_distance.getFilteredDist();
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
        if(left_trigger != 0 && !left_lock){
            if(right_trigger != 0){
                right_lock = true;
            }else{
                right_lock = false;
            }
            left_strafe = true;
        }else{
            left_strafe = false;
        }
        if(right_trigger != 0 && !right_lock){
            if(left_trigger != 0){
                left_lock = true;
            }else{
                left_lock = false;
            }
            right_strafe = true;
        }else{
            right_strafe = false;
        }
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
        }
        fast_drive = false;
        auto_move = true;
        target = distance;


    }
    public void PIDToDistance(double distance){
        if (target != distance){
            previousError = 0;
            integral = 0;
            distance_to_target = distance - getDistance();
            starting_motion_profile_time = timer.seconds();
            starting_pos = getDistance();
            auto_move = true;
            fast_drive = false;

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
            if(fast_drive){
                telemetry.addData("Direction", fast_drive_direction);
                telemetry.addData("Front Distance", front_distance);
                telemetry.addData("Reading Distance Value", getDistance());
                telemetry.addData("Target", target);
                telemetry.addData("Speed", fast_drive_speed);
                boolean completed = fast_drive_direction ? (front_distance ? ( getDistance() < target) : (getDistance() > target)) : (front_distance ? (getDistance() > target) : (getDistance() < target));
                if(completed){
                    fast_drive = false;
                    auto_move = false;
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontRight.setPower(0);
                    trafficLight.red(false);
                    trafficLight.green(true);
                    trafficLight.flashGreen(1, 1);

                }else {
                    trafficLight.red(true);
                    frontLeft.setPower(fast_drive_speed);
                    frontRight.setPower(fast_drive_speed);
                    backLeft.setPower(fast_drive_speed);
                    backRight.setPower(fast_drive_speed);
                }


            }
            else {
                if (motion_profile) {
                    error = getDistance() - (Control.motionProfile(MAX_VEL, MAX_ACCEL, MAX_DECEL, distance_to_target, timer.seconds() - starting_motion_profile_time) + starting_pos);
                } else {
                    error = getDistance() - target;
                }
                error *= front_distance ? 1 : -1;
                telemetry.addData("weird goofy ah error", error);

                // Calculate error
                double time_passed = timer.seconds() - time_stop;

                // Stop condition when error is sufficiently small or auto_move is disabled
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
                    // PID Calculations
                    double errorChange = (error - previousError);

                    // filter out hight frequency noise to increase derivative performance
                    currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
                    previousFilterEstimate = currentFilterEstimate;

                    integral += error * time_passed;  // Accumulate the error over time
                    double derivative = currentFilterEstimate / time_passed;

                    // PID output
                    double power = kP * error + kI * integral + kD * derivative;

                    // Limit power to a safe range (optional, depending on your motor controller)
                    power = Math.max(-0.5, Math.min(0.5, power));
                    power -= kF * Math.signum(power);
                    // Set motor power based on the PID output
                    frontLeft.setPower(power);
                    backLeft.setPower(power);
                    backRight.setPower(power);
                    frontRight.setPower(power);

                    // Update previous error
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
        if(RobotConstants.imu_init) telemetry.addData("Robot Heading", getHeading());
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