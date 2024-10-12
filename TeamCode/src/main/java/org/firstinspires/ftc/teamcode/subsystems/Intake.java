package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

@Config
public class Intake extends Subsystem {
    public static PIDFCoefficients armsPID = new PIDFCoefficients(0.002,0.0001,0.00006,0.0005);
    public NGMotor arm;
    public NGMotor slides;
    public static PIDFCoefficients slidesPID = new PIDFCoefficients(0.007, 0.0006, 0.0000385, 0.0005);

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
    public static double arm_at_0_ticks = 1400;
    public static double arm_at_45_ticks = 2010;
    public static double wrist_90 = 1;//TODO: make this work with the claw.

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        robotConstants = new RobotConstants();
        arm = new NGMotor(hardwareMap, telemetry, robotConstants.intakeMotor);
        arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        slides = new NGMotor(hardwareMap, telemetry, robotConstants.slidesMotor);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
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
        arm.move_async(targetPosition);
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
    public void setRotationPower(double power){ arm.setManualPower(power); }
    public void setAbsRotationPower(double power){ arm.setAbsPower(power);}
    public void setSlidePower(double power){
        slides.setManualPower(power);
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
        double arm_angle = (double) (arm.getCurrentPosition() - arm_at_0_ticks) / (arm_at_45_ticks - arm_at_0_ticks) * 45.0;
        arm_angle = 90 - arm_angle;
        double target_position = (90 - arm_angle - target_angle)/90.0 * wrist_90;

        return target_position;
    }
    @Override
    public void update() {
        arm.update();
        if(arm.getCurrentPosition() > feedforward_turning_point){
            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, -armsPID.f);
        }else{
            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);

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
        arm.telemetry();
    }

    @Override
    public void init() {
        arm.init();
        slides.init();
        openClaw();
        foldWrist();
    }
}
