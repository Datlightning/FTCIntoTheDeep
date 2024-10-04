package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;

public class MecaTank extends Subsystem {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private RobotConstants config;
    private Telemetry telemetry;
    private double MAX_DRIVE_SPEED = 1;
    public static double REAR_MIN_DISTANCE = 2.08;
    public static double FRONT_MIN_DISTANCE = 5.7;
    public static double distance_sensor_distance = 7.5;
    private boolean enable_front_stop = false;
    private boolean enable_rear_stop = false;
    private boolean left_strafe = false;
    private boolean right_strafe = false;
    private boolean left_lock = false;
    private boolean right_lock = false;
    public MecaTank(HardwareMap hardwareMap, Telemetry telemetry){
        config = new RobotConstants();
        frontLeft = hardwareMap.get(DcMotor.class, config.fl);
        frontRight = hardwareMap.get(DcMotor.class, config.fr);
        backLeft = hardwareMap.get(DcMotor.class, config.bl);
        backRight = hardwareMap.get(DcMotor.class, config.br);


        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        this.telemetry = telemetry;

    }

    private double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
   void set_individual_powers(double fl_power, double fr_power, double bl_power, double br_power){
        frontRight.setPower(fr_power);
        frontLeft.setPower(fl_power);
        backLeft.setPower(bl_power);
        backRight.setPower(br_power);
    }

    public void setPowers(double left_stick_y, double right_stick_y, double left_trigger, double right_trigger){
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


    @Override
    public void update() {

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