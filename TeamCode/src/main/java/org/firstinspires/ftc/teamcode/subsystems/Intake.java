package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double P = 0.0005, I = 0.0002, D = 0, F = 0.1;
    NGMotor motor;
    RobotConstants robotConstants;
    NGServo claw;
    NGServo wrist;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private boolean power_four_bar_enabled = false;
    private double target_angle = 0;

    public static double feedforward_turning_point = 0;
    public static double arm_at_0_ticks = 1750;
    public static double arm_at_45_ticks = 2302;
    public static double wrist_90 = 0.31;//TODO: make this work with the claw.

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        robotConstants = new RobotConstants();
        motor = new NGMotor(hardwareMap, telemetry, robotConstants.intakeMotor);
        motor.setPIDF(P,I,D,F);
        claw = new NGServo(hardwareMap, telemetry, robotConstants.claw_servo);
        wrist=  new NGServo(hardwareMap, telemetry, robotConstants.wrist_servo);
    }
    public void setFourBar(boolean state){
        power_four_bar_enabled = state;
    }
    public void setTargetAngle(double target_angle){
        this.target_angle = target_angle;
    }
    public void moveArm(int targetPosition){
        motor.move_async(targetPosition);
    }
    public void moveWrist(double wristPosition){
        wrist.setPosition(wristPosition);
    }
    public void moveClaw(double clawPosition){
        claw.setPosition(clawPosition);
    }
    public void setPower(double power){ motor.setPower(power); }
    public void setAbsPower(double power){ motor.setAbsPower(power);}
    public void openClaw() {
        claw.setPosition(robotConstants.claw_open);
    }
    public void closeClaw() {
        claw.setPosition(robotConstants.claw_closed);
    }
    public void foldWrist(){
        wrist.setPosition(robotConstants.wrist_folded);
    }
    public void extendWrist(){
        wrist.setPosition(robotConstants.wrist_extended);
    }

    public double calculateWristPosition(){
        double arm_angle = (double) (motor.getCurrentPosition() - arm_at_0_ticks) / (arm_at_45_ticks - arm_at_0_ticks) * 45.0;
        arm_angle = 90 - arm_angle;
        double target_position = (90 - arm_angle - target_angle)/90.0 * wrist_90;

        return target_position;
    }
    @Override
    public void update() {
        motor.update();
        if(motor.getCurrentPosition() > feedforward_turning_point){
            motor.setPIDF(P, I,D , -F);
        }else{
            motor.setPIDF(P, I,D , F);

        }
        if(power_four_bar_enabled){
            moveWrist(calculateWristPosition());
        }
    }

    @Override
    public void telemetry() {
        telemetry.addData("Claw Position", claw.getPosition());
        telemetry.addData("Wrist Position", claw.getPosition());
        motor.telemetry();
    }

    @Override
    public void init() {
        motor.init();
        claw.setPosition(robotConstants.claw_closed);
        wrist.setPosition(robotConstants.wrist_folded);

    }
}
