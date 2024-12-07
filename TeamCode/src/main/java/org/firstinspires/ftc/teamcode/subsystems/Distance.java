package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Distance extends Subsystem {
    private DistanceSensor sensorDistance;
    private double past_distance_reading = 0;
    private double current_dist = 0;
    private double filtered_position = 0;
    public static double filter_value = 0.65;
    private double delay = 0;

    private boolean on = false;
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
    public void setFilter(double filter){
        filter_value = filter;
    }
    public void setOn(boolean on){
        this.on = on;
    }
    @Override
    public void telemetry() {
        telemetry.addData(name + " is on: ", on);
        if(!on){
            return;
        }
        telemetry.addData(name + " Distance (in)", getDist());
        telemetry.addData(name + " Filtered Distance (in)", getFilteredDist());
    }
    public void update(){
        if(!on){
            return;
        }
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
    public class updateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetryPacket.put(name + " Filtered Distance", getFilteredDist());
            update();
            return true;
        }
    }
    public class waitAction implements Action {
        double target_distance = 0;
        boolean first= true;
        boolean direction = true;
        public waitAction(double target_distance){
            this.target_distance = target_distance;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                setOn(true);
                direction = getDist() < target_distance;
                first = false;
            }
            telemetryPacket.put(name + " Filtered Distance In Wait Action", getFilteredDist());
            boolean exit = direction ? getFilteredDist() < target_distance : getFilteredDist() > target_distance;
            setOn(exit);
            return exit;
        }
    }
    public Action waitAction(double target){
        return new waitAction(target);
    }
    public Action updateAction(){
        return new updateAction();
    }
}