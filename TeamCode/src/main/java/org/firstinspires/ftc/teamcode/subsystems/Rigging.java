package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Rigging extends Subsystem {
    public NGMotor rigging_motor;
    private Telemetry telemetry;

    public static int extend_height = 1000;
    public Rigging(HardwareMap hardwareMap, Telemetry telemetry){
        rigging_motor = new NGMotor(hardwareMap, telemetry, RobotConstants.riggingMotor);
        this.telemetry = telemetry;
    }
    public void extendRigging(){
        rigging_motor.move_async(extend_height);
    }


    @Override
    public void update() {
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
        rigging_motor.setMax(10000);
        rigging_motor.setMin(-10000);
        rigging_motor.setPID(1,0,0);

    }
}
