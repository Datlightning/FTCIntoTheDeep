package org.firstinspires.ftc.teamcode.library;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.concurrent.TimeUnit;

public class NGMotor extends Subsystem {
    DcMotorEx pid_motor;
    public double targetPos;
    public static double P = 0.0005, I = 0.0002, D = 0;
    public static double F = 0;
    double error, lastError;
    int startPos = Integer.MAX_VALUE;
    boolean direction = true;
    private boolean reversed_encoder = false;
    public boolean manual = false;
    private double powerReduction = 1;
    private double conversion = -1;
    private boolean reached = false;
    private double minPower = 0;
    private ElapsedTime timer;
    private double time_passed = 0;
    private int maxHardstop = 10000;
    private String name = "";
    private int minHardstop = 0;
    private double holding_power = 0;
    private double max_integral = 0;
    private double out = 0;
    private double completed_time = 0;
    private double MAX_POWER = 1;
    private int SLOW_POS = 0;
    private boolean SLOW = false;
    private double integralSum = 0;



    Telemetry telemetry;

    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name) {
        this.telemetry = telemetry;
        this.name = name;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);



        timer = new ElapsedTime();

    }
    public NGMotor(HardwareMap hardwareMap, Telemetry telemetry, String name, ElapsedTime timer) {
        this.telemetry = telemetry;
        this.name = name;
        pid_motor = hardwareMap.get(DcMotorEx.class, name);

        pid_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pid_motor.setDirection(DcMotor.Direction.FORWARD);



        this.timer = timer;

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
        this.max_integral = max_integral;
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

    public boolean isBusy(){
        return !(reached && timer.time(TimeUnit.SECONDS) - completed_time > 0.125);
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
    @Override
    public void telemetry(){
        telemetry.addData(name + " Power", getPower());
        telemetry.addData(name + "'s Position", getCurrentPosition());
        telemetry.addData(name + "'s Target Position", targetPos);
        telemetry.addData(name + "'s Max Hardstop", maxHardstop);
        telemetry.addData(name + "'s Min Hardstop", minHardstop);
        telemetry.addData(name + "'s direction", direction);
        telemetry.addData(name + " Exceeding Constraints", exceedingConstraints());
        telemetry.addData(name + " Busy Status", isBusy());
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
        }
        if(power == 0 && manual){
            targetPos = pid_motor.getCurrentPosition();
            manual = false;

        }
    }
    public void setPower(double power) {
        if(!exceedingConstraints(power)) {
            pid_motor.setPower(power + holding_power);
        }else{
            pid_motor.setPower(holding_power);
        }

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

    public void move_sync(double target, double timeoutS, double MOTOR_POWER) {
        double slidesPosition = getCurrentPosition();
        double Kp = P;
        double Ki = I;
        double Kd = D;

        double reference = target;

        double integralSum = 0;

        double currentTime = timer.time();
        while (timer.time() - currentTime < timeoutS && !exceedingConstraints()) {

            error = reference - slidesPosition;

            // rate of change of the error

            // sum of all error over time
            out = (Kp * error);
            integralSum = integralSum + (error * timer.seconds());
            if (integralSum <= max_integral){
                out += (Ki * integralSum) ;
            }



            if(timer.seconds() != 0) {
                double derivative = (error - lastError) / timer.seconds();
                out += Kd * derivative;
            }
            out += F;
            out *= (reversed_encoder ? -1 : 1);
            setPower(out * MOTOR_POWER);
            if(Math.abs(error) < 5){
                break;
            }

        }
        setPower(0);

    }

    public void move_async(double target) {
        targetPos = target;
        reached = false;
        integralSum = 0;
    }


    public double getPower(){
        return pid_motor.getPower();
    }

    public void update() {
        if(manual){
            return;
        }
        // obtain the encoder position
        // calculate the error
        error = targetPos - getCurrentPosition();

        // rate of change of the error
        time_passed = timer.seconds() - time_passed;
        // sum of all error over time
        out = (P * error);
        integralSum = integralSum + (error * time_passed);
        if (integralSum <= max_integral){
            out += (I * integralSum) ;
        }



        if(time_passed != 0) {
            double derivative = (error - lastError) / time_passed;
            out += D * derivative;
        }
        out += Math.copySign(F, out) ;
        out *= (reversed_encoder ? -1 : 1);

        setPower(out* MAX_POWER);

        if(Math.abs(error) < 20 && !reached){
            reached = true;
            completed_time = timer.time(TimeUnit.SECONDS);
        }
        lastError = error;

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