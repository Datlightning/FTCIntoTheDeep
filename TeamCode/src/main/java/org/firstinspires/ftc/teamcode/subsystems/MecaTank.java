package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;

import java.util.concurrent.TimeUnit;

@Config
public class MecaTank extends Subsystem {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private RobotConstants config;

    private BNO055IMU imu; // IMU for field-centric control

    private Distance distance;
    private Telemetry telemetry;
    private double MAX_DRIVE_SPEED = 1;
    private boolean left_strafe = false;
    private boolean right_strafe = false;
    private boolean left_lock = false;
    private boolean right_lock = false;
    private boolean force_exit = false;
    public static double kP = 0.05;  // Proportional constant

    private double target = 0;
    private boolean auto_move = false;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

    }
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

    }
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, BNO055IMU imu){
        frontLeft = hardwareMap.get(DcMotor.class, RobotConstants.fl);
        frontRight = hardwareMap.get(DcMotor.class, RobotConstants.fr);
        backLeft = hardwareMap.get(DcMotor.class, RobotConstants.bl);
        backRight = hardwareMap.get(DcMotor.class, RobotConstants.br);
        this.imu = imu;
        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

    }


    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    private double getHeading() {
        return imu.getAngularOrientation().firstAngle;
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
    public void PIDToDistance(double distance){
        target = distance;
        auto_move = true;
    }
    @Override
    public void update() {
        if(force_exit){
            auto_move = false;
            force_exit = false;
        }
        if(auto_move){
            double error =  distance.getDist() - target;  // Calculate error
            if (Math.abs(error) < 0.5 || !auto_move) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
                auto_move = false;
            }
            double power = kP * error + Math.copySign(0.1, error);
// Set motor power proportionally to the error
            frontLeft.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            frontRight.setPower(power);

        }
    }
    public boolean isBusy(){
        return auto_move;
    }
    @Override
    public void telemetry() {

        telemetry.addData("Front Right Motor Power", frontRight.getPower());
        telemetry.addData("Front Left Motor Power", frontLeft.getPower());
        telemetry.addData("Back Right Motor Power", backRight.getPower());
        telemetry.addData("Back Left Motor Power", backLeft.getPower());
    }

    @Override
    public void init() {

    }

    public void setMaxPower(double v) {
        MAX_DRIVE_SPEED = v;
    }
}