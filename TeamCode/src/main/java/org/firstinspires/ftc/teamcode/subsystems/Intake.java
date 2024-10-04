package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double[] armsPID = {0.0026, 0.0002, 0.00002, 0.005};
    NGMotor motor;
    NGMotor slides;
    public static double[] slidesPID = {0.0026, 0.0002, 0.00002, 0.005};

    RobotConstants robotConstants;
    NGServo claw;
    NGServo wrist;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private boolean claw_open = false;
    private boolean wrist_open =false;
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
        motor.setPIDF(armsPID[0], armsPID[1], armsPID[2], armsPID[3]);
        slides = new NGMotor(hardwareMap, telemetry, robotConstants.slidesMotor);
        slides.setPIDF(slidesPID[0], slidesPID[1], slidesPID[2], slidesPID[3]);
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
    public void moveSlides(int targetPosition){
        slides.move_async(targetPosition);
    }
    public void setRotationPower(double power){ motor.setPower(power); }
    public void setAbsRotationPower(double power){ motor.setAbsPower(power);}
    public void setSlidePower(double power){
        slides.setPower(power);
    }
    public void setAbsSlidePower(double power){
        slides.setAbsPower(power);
    }
    public void openClaw() {
        claw.setPosition(robotConstants.claw_open);claw_open = true;
    }
    public void setClawPWM(boolean on){
        if(on){
            claw.enableServo();
            return;
        }
        claw.disableServo();
    }
    public void setWristPWM(boolean on){
        if(on){
            wrist.enableServo();
            return;
        }
        wrist.disableServo();
    }
    public boolean WristPWMOn(){
        return wrist.isPWMEnabled();
    }
    public boolean ClawPWMOn(){
        return claw.isPWMEnabled();
    }
    public void closeClaw() {
        claw.setPosition(robotConstants.claw_closed); claw_open=false;
    }
    public void foldWrist(){
        wrist.setPosition(robotConstants.wrist_folded); wrist_open=false;
    }
    public void extendWrist(){
        wrist.setPosition(robotConstants.wrist_extended); wrist_open=true;
    }
    public boolean isClawOpen(){
        return claw_open;
    }
    public boolean isWristOpen(){
        return wrist_open;
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
            motor.setPIDF(armsPID[0], armsPID[1], armsPID[2], -armsPID[3]);
        }else{
            motor.setPIDF(armsPID[0], armsPID[1], armsPID[2], armsPID[3]);

        }
        if(power_four_bar_enabled){
            moveWrist(calculateWristPosition());
        }
        slides.update();
    }

    @Override
    public void telemetry() {
        claw.telemetry();
        wrist.telemetry();
        slides.telemetry();
        motor.telemetry();
    }

    @Override
    public void init() {
        motor.init();
        slides.init();
        openClaw();
        foldWrist();
    }
}
