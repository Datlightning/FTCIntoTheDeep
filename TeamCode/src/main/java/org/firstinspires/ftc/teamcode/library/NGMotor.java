package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

public class NGMotor extends Subsystem {
    DcMotorEx pid_motor;

    private boolean on = true;
    public int targetPos = 0;
    private int startPos = 0;
    public static double P = 0.0005, I = 0.0002, D = 0;
    public static double F = 0;
    double error, lastError;
    boolean direction = true;
    private boolean reversed_encoder = false;
    public boolean manual = false;
    private double powerReduction = 1;
    private double conversion = -1;
    private boolean reached = false;
    private boolean useMotionProfile = false;
    private double minPower = 0;
    private int distance = 0;
    private ElapsedTime timer;
    private double time_passed = 0, time_stop = 0, time_started = 0;
    private int maxHardstop = 10000;
    private String name = "";
    private int minHardstop = 0;
    private double holding_power = 0;
    private double max_integral_component = 0.3;
    private double out = 0;
    private double completed_time = 0;
    private double MAX_POWER = 1;
    private int SLOW_POS = 0;
    private boolean SLOW = false;
    private int reached_range = 10;
    private double integralSum = 0;

    private double MAX_VEL = 1;
    private double MAX_ACCEL = 1;

    private double MAX_DECEL = -1;



    Telemetry telemetry;

    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        this.name = name;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);



        timer = new ElapsedTime();

    }
    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPower){
        pid_motor.setZeroPowerBehavior(zeroPower);
    }
    public void enableMotor(boolean on){
        this.on = on;
    }
    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name, ElapsedTime timer) {
        this.telemetry = telemetry;
        this.name = name;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);



        this.timer = timer;

    }
    public void resetEncoder(){
        pid_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setReachedRange(int range){
        reached_range = range;
    }
    public void setPID(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = 0;
    }
    public void setPIDF(double P, double I, double D, double F){
        setPID(P,I,D);
        this.F = F;
    }

    public void setMaxIntegral(double max_integral){
        this.max_integral_component = max_integral;
    }
    public void setDirection(DcMotor.Direction power){
        pid_motor.setDirection(power);
    }

    public void setMin(int min){
        minHardstop = min;
    }

    public void setMax(int max){
        maxHardstop = max;
    }
    public void setMaxVelocity(double speed){
        MAX_VEL = speed;
    }
    public void setMaxAcceleration(double accel){
        MAX_ACCEL = accel;
    }

    public void setMaxDeceleration(double decel){ MAX_DECEL = decel;}
    public boolean isBusy(){
        return !(reached);
    }
    public boolean isCompletedFor(double time){
        return !(reached && timer.time(TimeUnit.SECONDS) - completed_time > time);

    }
    @Override
    public void init(){
        setPower(0);
        pid_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pid_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getCurrent(){
        return pid_motor.getCurrent(CurrentUnit.MILLIAMPS);
    }
    public double getCurrentAlert(){
        return pid_motor.getCurrentAlert(CurrentUnit.AMPS);
    }
    public double getVelocity(){
        return pid_motor.getVelocity();
    }
    @Override
    public void telemetry(){
        telemetry.addData(name + " Power", getPower());
        telemetry.addData(name + "'s Position", getCurrentPosition());
        telemetry.addData(name + "'s Target Position", targetPos);
        telemetry.addData(name + "'s Max Hardstop", maxHardstop);
        telemetry.addData(name + "'s Min Hardstop", minHardstop);
        telemetry.addData(name + " Exceeding Constraints", exceedingConstraints());
        telemetry.addData(name + " Busy Status", isBusy());
        telemetry.addData(name + "Current Draw", getCurrent());
        telemetry.addData(name + "Current Cooked", getCurrentAlert());

        telemetry.addData(name + "Velocity", getVelocity());
        telemetry.addData("Out Power", out);


    }


    public boolean exceedingConstraints(){
        boolean over_max = getCurrentPosition() > maxHardstop;
        boolean under_min = getCurrentPosition() < minHardstop;
        return getPower() > 0 ? over_max : under_min;

    }
    public boolean exceedingConstraints(double power){
        boolean over_max = getCurrentPosition() > maxHardstop;
        boolean under_min = getCurrentPosition() < minHardstop;
        if (!reversed_encoder) {
            return power > 0 ? over_max : under_min;
        }
        return power < 0 ? over_max : under_min;

    }

    public void setManualPower(double power){
        if(power != 0){
            manual = true;
            setPower(power);
        }
        if(power == 0 && manual){
            move_async(pid_motor.getCurrentPosition());
            manual = false;
            setPower(0);

        }
    }
    public void setPower(double power) {
        if(!exceedingConstraints(power)) {
            pid_motor.setPower(power + holding_power);
        }
        else{
            pid_motor.setPower(holding_power);
        }

    }
    public void setUseMotionProfile(boolean on){
        useMotionProfile = on;
    }
    public void setReversedEncoder(boolean reversed){
        this.reversed_encoder = reversed;
    }
    public int getCurrentPosition() {
        return pid_motor.getCurrentPosition();
    }
    public void setAbsPower(double power){
        pid_motor.setPower(power);

    }

    public void move_async(int target) {
        target = Math.min(target, maxHardstop);
        target = Math.max(target, minHardstop);

        if(targetPos != target){
            reached = false;
            integralSum = 0;
            time_stop = timer.seconds();
            time_started = timer.seconds();
            distance = target - getCurrentPosition();
            startPos = getCurrentPosition();
            telemetry.addData("Distnace", distance);
            telemetry.addData("Target", target);


        }
        targetPos = target;

    }


    public double getPower(){
        return pid_motor.getPower();
    }

    public void update() {
        if(manual){
            return;
        }
        if(!on){
            return;
        }
//        telemetry.addData(name + " P", P);
//        telemetry.addData(name + " I",I);
//        telemetry.addData(name + " D", D);
//        telemetry.addData(name + " F", F);
        // Obtain the encoder position and calculate the error
        if(useMotionProfile){
            if(MAX_DECEL == -1){
                error = Control.motionProfile(MAX_ACCEL, MAX_VEL, distance, timer.seconds() - time_started) + startPos - getCurrentPosition();

            }else{
                error = Control.motionProfile( MAX_VEL, MAX_ACCEL, MAX_DECEL, distance, timer.seconds() - time_started) + startPos - getCurrentPosition();

            }
            telemetry.addData(name + " Expected Velocity",  Control.motionProfileVelo(MAX_ACCEL, MAX_VEL, distance, timer.seconds() - time_started));

        }else {
            error = targetPos - getCurrentPosition();
        }

// Calculate time passed
        time_passed = timer.seconds() - time_stop;
          // Update time_stop early

// Proportional term
        out = P * error;

// Integral term with windup protection
        integralSum += error * time_passed;
        if (I * integralSum <= max_integral_component){
            out += I * integralSum;
        } else {
            out += max_integral_component;
        }

// Derivative term, ensuring time_passed is not zero
        if(time_passed > 0) {
            double derivative = (error - lastError) / time_passed;
            out += D * derivative;
        }
// Feedforward term
        out += F;
// Check if the target has been reached (based on an error threshold)
        if(Math.abs(getCurrentPosition() - targetPos) < 10) {
            reached = true;
        }
// Set motor power output
        pid_motor.setPower(out);
// Update lastError for the next iteration
        lastError = error;
        time_stop = timer.seconds();
    }

    public void setMaxPower(double power) {
        MAX_POWER = power;
    }



//    public double getBatteryVoltage() {
//        double result = Double.POSITIVE_INFINITY;
//        for (VoltageSensor sensor : hardware.voltageSensor) {
//            double voltage = sensor.getVoltage();
//            if (voltage > 0) {
//                result = Math.min(result, voltage);
//            }
//        }
//        return result;
//    }
}