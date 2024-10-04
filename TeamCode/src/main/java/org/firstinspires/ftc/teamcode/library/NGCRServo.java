package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class NGCRServo extends Subsystem{
    CRServoImplEx servo;
    Telemetry telemetry;
    private double speed = 0;
    private String name;
    private double init_power = 0;
    private boolean PWMEnabled = true;



    public NGCRServo(HardwareMap hardwareMap, Telemetry telemetry, String name){
        servo =  hardwareMap.get(CRServoImplEx.class, name);
        this.telemetry = telemetry;
        this.name = name;
    }
    public void setInitPower(double position){
        init_power = position;
    }
    public void disableServo(){
        servo.setPwmDisable();
        PWMEnabled = false;
    }
    public void enableServo(){
        servo.setPwmEnable();
        PWMEnabled = true;
    }
    public void setPower(double position){
        servo.setPower(position);
        speed = 0;
    }
    public boolean isPWMEnabled(){
        return PWMEnabled;
    }
    public double getPower(){
        return servo.getPower();
    }
    @Override
    public void update() {

    }

    @Override
    public void telemetry() {
        telemetry.addData(name + " power: ", servo.getPower());
        telemetry.addData(name + " enabled: ", PWMEnabled);
        if(speed != 0){
            telemetry.addData(name + " speed: ", speed);
        }
    }

    @Override
    public void init() {
        servo.setPower(init_power);
    }
}
