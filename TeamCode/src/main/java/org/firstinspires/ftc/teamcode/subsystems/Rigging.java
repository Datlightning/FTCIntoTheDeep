package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Rigging extends Subsystem {
    public NGMotor rigging_motor;
    private Telemetry telemetry;

    public static PIDCoefficients PID = new PIDCoefficients(0.005,0,0);
    public static int unlatch_height = 6000;
    public static int full_extend = 12400;

    private ElapsedTime timer;
    public static int off_ground = 5500;

    DigitalChannel magnet_sensor;
    public Rigging(HardwareMap hardwareMap, Telemetry telemetry){
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor);
        rigging_motor.setDirection(DcMotor.Direction.REVERSE);
        magnet_sensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnet_sensor2);
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);
        timer = new ElapsedTime();
        this.telemetry = telemetry;
    }
    private boolean magnet_activated(){
        return !magnet_sensor.getState();
    }
    public void reset(){
        if (magnet_activated()){
            return;
        }
        rigging_motor.setAbsPower(-1);
        double current_time = timer.seconds();
        while (!magnet_activated() && timer.seconds() - current_time < 5){
            rigging_motor.setAbsPower(-1);
        }
        rigging_motor.setAbsPower(0);
        rigging_motor.resetEncoder();
    }
    public class updateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }
    public Action updateAction(){
        return new updateAction();
    }

    public Rigging(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this(hardwareMap, telemetry);
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor, timer);
        rigging_motor.setDirection(DcMotor.Direction.REVERSE);
        this.timer = timer;

    }
    public void unlatchHooks(){
        rigging_motor.move_async(unlatch_height);
    }
    public void raiseHooks(){
        rigging_motor.move_async(full_extend);
    }
    public void rig(){
        rigging_motor.move_async(off_ground);
    }
    public void setManualPower(double power){
        if (power < 0 && magnet_activated()){
            rigging_motor.setManualPower(0);
            return;
        }
        rigging_motor.setManualPower(power);
    }


    @Override
    public void update() {
        rigging_motor.setExternalDownHardstop(magnet_activated());
        rigging_motor.setPID(PID.p, PID.i, PID.d);
        rigging_motor.update();
    }

    @Override
    public void telemetry() {
        rigging_motor.telemetry();
    }

    @Override
    public void init() {
        rigging_motor.init();
        rigging_motor.setUseMotionProfile(false);
        rigging_motor.setMax(20000);
        rigging_motor.setMin(-20000);

    }
}
