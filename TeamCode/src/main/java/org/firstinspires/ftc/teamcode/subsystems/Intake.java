package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.NGMotor;
import org.firstinspires.ftc.teamcode.library.NGServo;
import org.firstinspires.ftc.teamcode.library.Subsystem;

import java.util.ArrayList;

@Config
public class Intake extends Subsystem {
    public static PIDFCoefficients armsPID = new PIDFCoefficients(0.0018,0.00006,0.00000004,0.0008);
    public static PIDFCoefficients armsLevelPID = new PIDFCoefficients(0.002,0.00006,0,0.000025);
    public NGMotor arm;
    public NGMotor slides;
    public static PIDFCoefficients slidesPID = new PIDFCoefficients(0.005, 0.00001, 0.000003, 0.0005);

    RobotConstants robotConstants;
    NGServo claw;
    NGServo wrist;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private boolean claw_open = false;
    private boolean wrist_open =false;
    private boolean power_four_bar_enabled = false;
    private boolean forward_delay = false;
    private boolean backward_delay = false;

    private double target_angle = 0;//To the y-axis
    private double target_height = 0;//To the floor
    private ArrayList<Integer> slideTargetPositions = new ArrayList<>();
    private ArrayList<Integer> armTargetPositions = new ArrayList<>();



    public static double feedforward_turning_point = 0;
    public static int arm_at_0_ticks = 150;
    public static int arm_at_90_ticks = 1410;
    public static double wrist_90 = .33;
    public static double wrist_180 = 0.69;
    public static double slide_ticks_to_inches = 0.01875;
    public static double slide_starting_length = 13.25;
    public static double slide_starting_height = 9;
    public static double slide_width = 3.25;
    public static double claw_height = 10;



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
        arm.setReachedRange(50);
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        robotConstants = new RobotConstants();
        arm = new NGMotor(hardwareMap, telemetry, robotConstants.intakeMotor, timer);
        arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        slides = new NGMotor(hardwareMap, telemetry, robotConstants.slidesMotor, timer);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        claw = new NGServo(hardwareMap, telemetry, robotConstants.claw_servo);
        claw.mountTimer(timer);
        wrist =  new NGServo(hardwareMap, telemetry, robotConstants.wrist_servo);
        wrist.mountTimer(timer);
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
        if(targetPosition < slides.getCurrentPosition()){
            slides.setMaxAcceleration(4000);
        }else{
            slides.setMaxAcceleration(9000);
        }
        slides.move_async(targetPosition);
    }
    public void setRotationPower(double power){ arm.setManualPower(power); }
    public void setTargetHeight(double target_height){ this.target_height = target_height; }
    public void setAbsRotationPower(double power){ arm.setAbsPower(power);}
    public void setSlidePower(double power){slides.setManualPower(power);}
    public void setAbsSlidePower(double power){
        slides.setAbsPower(power);
    }
    public void openClaw() {claw.setPosition(robotConstants.claw_fully_open);claw_open = true;}
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
    public void plowClaw(){
        claw.setPosition(robotConstants.claw_open); claw_open=true;
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
    public double getArmAngle(){
        return (double) (arm.getCurrentPosition() - arm_at_0_ticks) / (arm_at_90_ticks - arm_at_0_ticks) * 90.0;
    }
    public double calculateWristPosition(){
        double arm_angle = getArmAngle();
        double target_position = (90 - arm_angle - target_angle)/90.0 * (wrist_180 - wrist_90) + wrist_90;
        return target_position;
    }
    public double calculateSlideLength(int slide_position){
        return Math.sqrt(Math.pow(slide_starting_length + slide_position * slide_ticks_to_inches,2) + Math.pow(slide_width,2));
    }
    public int calculateArmPosition(int slide_position){
        double arm_angle = Math.asin((target_height + claw_height - slide_starting_height) / (calculateSlideLength(slide_position)));
        int arm_angle_in_ticks = (int) (arm_angle / Math.toRadians(90.0) * (arm_at_90_ticks - arm_at_0_ticks)) + arm_at_0_ticks;

        return arm_angle_in_ticks;
    }
    public void moveSlidesWithDelay(int increment){
        if(backward_delay){
            slideTargetPositions.clear();
            armTargetPositions.clear();
        }
        int lastPosition ;

        if(slideTargetPositions.isEmpty()) {
            slideTargetPositions.add(slides.targetPos + increment);
            lastPosition = slides.targetPos;
        } else {
            lastPosition = slideTargetPositions.get(slideTargetPositions.size() - 1);
            slideTargetPositions.add(lastPosition + increment);
        }
        int arm_pos = calculateArmPosition(lastPosition + increment);
        armTargetPositions.add(arm_pos);
        moveArm(arm_pos);
        forward_delay = true;
        backward_delay = false;
    }
    public void moveArmWithDelay(int increment){
        if(forward_delay){
            slideTargetPositions.clear();
            armTargetPositions.clear();
        }
        slideTargetPositions.add(slides.targetPos + increment);

        int arm_pos = calculateArmPosition(slides.targetPos + increment);
        armTargetPositions.add(arm_pos);
        moveSlides(slides.targetPos + increment);
        forward_delay = false;
        backward_delay = true;
    }
    @Override
    public void update() {
//        if(arm.getCurrentPosition() > feedforward_turning_point){
//            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, -armsPID.f);
//        }else{
//            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
//
//        }
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        if(arm.getCurrentPosition() > 330){
            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        }else{
            arm.setPIDF(armsLevelPID.p, armsLevelPID.i, armsLevelPID.d, Math.cos(Math.toRadians(getArmAngle())) * calculateSlideLength(slides.getCurrentPosition()) * armsLevelPID.f);
        }
        if(power_four_bar_enabled){
            moveWrist(calculateWristPosition());
        }
        if(forward_delay && slideTargetPositions.size() > 0 && arm.getCurrentPosition() <= armTargetPositions.get(0)){
            armTargetPositions.remove(0);
            moveSlides(slideTargetPositions.remove(0));
        }else if(slideTargetPositions.size() == 0){
            forward_delay = false;
        }
        if(backward_delay && armTargetPositions.size() > 0 && slides.getCurrentPosition() <= slideTargetPositions.get(0)){
            slideTargetPositions.remove(0);
            moveArm(armTargetPositions.remove(0));
        }else if(armTargetPositions.size() == 0){
            backward_delay = false;
        }
//        telemetry.addData("armTargetPositions", armTargetPositions.toString());
//        telemetry.addData("slideTargetPositions", slideTargetPositions.toString());
        arm.update();
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
        arm.setMaxAcceleration(3000);
        arm.setMaxVelocity(3900);
        wrist.setMin(0.19);
        wrist.setMax(0.95);
        slides.init();
        slides.setUseMotionProfile(true);
        slides.setMax(1400);
        slides.setMaxAcceleration(9000);
        slides.setMaxVelocity(10000);

        openClaw();
        foldWrist();
    }
}
