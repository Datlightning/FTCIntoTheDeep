package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Distance extends Subsystem {
    private DistanceSensor sensorDistance;

    private double offset = 0;
    private double past_distance_reading = 0;
    private double current_dist = 0;
    private double filtered_position = 0;
    public static double filter_value = 0.65;
    private double delay = 0;
    private ElapsedTime timer;
    private String name = "";
    Telemetry telemetry;
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, String name){
        this.telemetry = telemetry;
        timer = new ElapsedTime();

        sensorDistance = hardwareMap.get(DistanceSensor.class, name);
        this.name = name;

    }
    public Distance(HardwareMap hardwareMap, Telemetry telemetry, String name, ElapsedTime timer){
        this.telemetry = telemetry;
        this.timer = timer;
        this.name = name;
        sensorDistance = hardwareMap.get(DistanceSensor.class, name);

    }
    public double getDist() {
        return sensorDistance.getDistance(DistanceUnit.INCH);
    }
    public double getDistFromRobotEdge(){
        return getDist() - offset;
    }
    public void setOffset(double offset){
        this.offset = offset;
    }
    public void setFilter(double filter){
        filter_value = filter;
    }
    @Override
    public void telemetry() {
        telemetry.addData(name + " Distance (in)", getDist());
        telemetry.addData(name + " Filtered Distance (in)", getFilteredDist());
    }
    public void update(){
        if(timer.seconds() - delay > 0.005){
            past_distance_reading = current_dist;
            current_dist = getDist();
            if(Math.abs(past_distance_reading - current_dist) > 1){
                filtered_position = current_dist;
            }else {
                filtered_position = filter_value * current_dist + (1 - filter_value) * past_distance_reading;
            }
            delay = timer.seconds();
        }
    }
    public double getFilteredDist(){
        return filtered_position;
    }
    @Override
    public void init() {
        return;
    }
}