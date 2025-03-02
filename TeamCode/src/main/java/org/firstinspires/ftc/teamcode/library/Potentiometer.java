package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Potentiometer extends AlternateEncoder{
    AnalogInput pot;
    double currentVoltage;
    Telemetry telemetry;

    int position_at_90 = 0;
    int position_at_0 = 0;

    double real_zero_degrees = -2.2;

    public Potentiometer(HardwareMap hardwareMap, Telemetry telemetry, String name){
        this.telemetry = telemetry;
        pot = hardwareMap.get(AnalogInput.class, name);

    }
    public double getVoltage(){
        return pot.getVoltage();
    }
    public void set90Position(int position){
        position_at_90 = position;
    }
    public void set0Position(int position){
        position_at_0 = position;
    }
    public void setRealZeroDegrees(double degrees){
        real_zero_degrees = degrees;
    }
    public double getAngle(){
        if(getVoltage() == 0){
            return 270;
        }
        double voltage = getVoltage();
        double theta = (
                (270*voltage + 445.5) -
                Math.sqrt(
                        (270*voltage + 445.5) * (270*voltage + 445.5)
                                - 4 * voltage * (-36450 * voltage + 120285)
                )
        );
        return (270 - theta/(2 * voltage)) - real_zero_degrees;

    }
    @Override
    public int getCurrentPosition() {
        return (int) ((position_at_90 - position_at_0) * getAngle() / 90.0 ) + position_at_0;
    }
}
