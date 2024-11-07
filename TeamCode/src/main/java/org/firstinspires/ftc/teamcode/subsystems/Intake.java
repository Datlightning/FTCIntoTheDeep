package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    public static PIDFCoefficients armsPID = new PIDFCoefficients(0.003,0.00006,0.00002,0.0008);
    public static PIDFCoefficients armsLevelPID = new PIDFCoefficients(0.0018,0,0.00003,0.0002);
    public NGMotor arm;
    public NGMotor slides;
    public static PIDFCoefficients slidesPID = new PIDFCoefficients(0.005, 0.00001, 0.000003, 0.0005);
    DigitalChannel magnet_sensor;

    NGServo claw;
    NGServo wrist;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;

    public Distance distance;
    private boolean claw_open = false;
    private boolean wrist_open =false;
    private boolean power_four_bar_enabled = false;
    private boolean forward_delay = false;
    private boolean backward_delay = false;
    private boolean use_fast_pid = false;

    private int slide_stuck_offset = 0;
    private boolean level_on = false;
    private double target_angle = 0;//To the y-axis
    private double target_height = 0;//To the floor
    private ArrayList<Integer> slideTargetPositions = new ArrayList<>();
    private ArrayList<Integer> armTargetPositions = new ArrayList<>();



    public static double feedforward_turning_point = 0;
    public static int arm_at_0_ticks = 180;
    public static int arm_at_90_ticks = 1380;
    public static double wrist_90 = .2;
    public static double wrist_180 = 0.56;
    public static double slide_ticks_to_inches = 0.0171875;
    public static double slide_starting_length = 13;
    public static double slide_starting_height = 9.125;
    public static double slide_width = 3.125;
    public static double claw_height = 8;
    public static int offset = 0;
    public static int offset_distance_to_0 = -57;

    public static double DISTANCE_FILTER = 0.6;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        arm = new NGMotor(hardwareMap, telemetry, RobotConstants.intakeMotor);
        arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        arm.setDirection(DcMotor.Direction.REVERSE);
        timer = new ElapsedTime();
        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);
        magnet_sensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnet_sensor);
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);
        slides = new NGMotor(hardwareMap, telemetry, RobotConstants.slidesMotor);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        claw = new NGServo(hardwareMap, telemetry, RobotConstants.claw_servo);
        wrist =  new NGServo(hardwareMap, telemetry, RobotConstants.wrist_servo);
        arm.setMin(-500);
        arm.setReachedRange(50);
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        magnet_sensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnet_sensor);
        magnet_sensor.setMode(DigitalChannel.Mode.INPUT);
        this.timer = timer;
        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);
        arm = new NGMotor(hardwareMap, telemetry, RobotConstants.intakeMotor, timer);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        slides = new NGMotor(hardwareMap, telemetry, RobotConstants.slidesMotor, timer);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        claw = new NGServo(hardwareMap, telemetry, RobotConstants.claw_servo);
        claw.mountTimer(timer);
        wrist =  new NGServo(hardwareMap, telemetry, RobotConstants.wrist_servo);
        wrist.mountTimer(timer);
    }
    private boolean magnet_activated(){
        return !magnet_sensor.getState();
    }
    public void useFastPID(boolean on){
        use_fast_pid = on;
    }
    public void calculateOffset(){
        arm.setAbsPower(0.1);
        double time_offset = timer.time();
        while(!magnet_activated()){
            telemetry.addData("Position",arm.getCurrentPosition());
            telemetry.update();
            if(timer.time() - time_offset > 2){

                return;
            }
        }
        offset = arm.getCurrentPosition() - arm_at_0_ticks + offset_distance_to_0;
        telemetry.addData("Offset", offset);

        arm.setAbsPower(0);

    }
    public int getOffset(){
        return offset;
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
    public void openClaw() {claw.setPosition(RobotConstants.claw_fully_open);claw_open = true;}

    public Action score(){
        return new SequentialAction(
                armAction(1450),
                new InstantAction(this::openClaw),
                armAction(1400),
                new InstantAction(() -> moveWrist(RobotConstants.floor_pickup_position + 0.1)),
                slideAction(0));
    }
    public Action scoreLast(){
        return new SequentialAction(
                armAction(1450),
                new InstantAction(this::openClaw),
                new InstantAction(() -> moveWrist(RobotConstants.floor_pickup_position + 0.1))
        );
    }
    public Action grab(){
        return new SequentialAction(
                new InstantAction(this::closeClaw),
                new SleepAction(0.3)
        );
    }
    public Action grab(double position){
        return new SequentialAction(
                new InstantAction(() -> moveClaw(position)),
                new SleepAction(0.3)
        );
    }
    public Action raiseArm(){
        return new SequentialAction(
                armAction(1400 ,600),
                new ParallelAction(
//                        slideAction(1400),
                        armAction(1400)
                ),
                new InstantAction(() -> moveWrist(0.9))
        );
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
        claw.setPosition(RobotConstants.claw_closed); claw_open=false;
    }
    public void plowClaw(){
        claw.setPosition(RobotConstants.claw_open); claw_open=true;
    }
    public void enableLevel(boolean on){
        level_on = on;
    }
    public void foldWrist(){
        wrist.setPosition(RobotConstants.wrist_folded); wrist_open=false;
    }
    public void extendWrist(){
        wrist.setPosition(RobotConstants.wrist_extended); wrist_open=true;
    }
    public boolean isClawOpen(){
        return claw_open;
    }
    public boolean isWristOpen(){
        return wrist_open;
    }
    public double getArmAngle(){

        return (double) (arm.getCurrentPosition()  - arm_at_0_ticks -  offset ) / (arm_at_90_ticks - arm_at_0_ticks) * 90.0;
    }
    public double calculateWristPosition(){
        double arm_angle = getArmAngle();
        double target_position = (90 - arm_angle - target_angle)/90.0 * (wrist_180 - wrist_90) + wrist_90;

        return target_position;
    }
    public double calculateSlideLengthNotTheHypot(int slide_position){
        return slide_starting_length + slide_position * slide_ticks_to_inches;
    }
    public double calculateSlideLength(int slide_position){
        telemetry.addData("SlideLength", slide_starting_length + slide_position * slide_ticks_to_inches);

        return Math.sqrt(Math.pow(slide_starting_length + slide_position * slide_ticks_to_inches,2) + Math.pow(slide_width,2));
    }
    public int calculateArmPosition(int slide_position){
        double arm_angle = Math.asin((target_height + claw_height - slide_starting_height) / (calculateSlideLength(slide_position)));
        int arm_angle_in_ticks = (int) (arm_angle / Math.toRadians(90.0) * (arm_at_90_ticks - arm_at_0_ticks)) + arm_at_0_ticks + offset;

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
        if(level_on){
           arm.setUseMotionProfile(false);
           arm.move_async(calculateArmPosition(slides.getCurrentPosition()));
        }else{
            arm.setUseMotionProfile(true);
        }
        distance.setFilter(DISTANCE_FILTER);
        slides.setPIDF(slidesPID.p, slidesPID.i, slidesPID.d, slidesPID.f);
        if(arm.getCurrentPosition() > 330 || use_fast_pid){
            arm.setPIDF(armsPID.p, armsPID.i, armsPID.d, armsPID.f);
        }else{
            arm.setPIDF(armsLevelPID.p, armsLevelPID.i, armsLevelPID.d, Math.cos(Math.toRadians(getArmAngle())) * calculateSlideLength(slides.getCurrentPosition()) * armsLevelPID.f);
        }
        if(power_four_bar_enabled){
            moveWrist(calculateWristPosition());
        }

        distance.update();
//        telemetry.addData("armTargetPositions", armTargetPositions.toString());
        arm.update();
        slides.update();
    }

    @Override
    public void telemetry() {
        distance.telemetry();
        telemetry.addData("Arm Angle", getArmAngle());
        claw.telemetry();
        wrist.telemetry();
        slides.telemetry();
        arm.telemetry();
    }
    public void init_without_encoder_reset(){

        arm.setMaxAcceleration(4000);
        arm.setMaxVelocity(4900);
        arm.setUseMotionProfile(true);
        wrist.setMin(0.19);
        wrist.setMax(0.95);
        slides.setUseMotionProfile(true);
        slides.setMax(1400);
        slides.setMaxAcceleration(9000);
        slides.setMaxVelocity(10000);
    }

    @Override
    public void init() {
        arm.init();
        arm.setMaxAcceleration(4000);
        arm.setMaxVelocity(4900);
        arm.setUseMotionProfile(true);
        wrist.setMin(0.19);
        wrist.setMax(0.95);
        slides.init();
        slides.setUseMotionProfile(true);
        slides.setMax(1400);
        slides.setMaxAcceleration(9000);
        slides.setMaxVelocity(10000);
        moveWrist(0.9);
        closeClaw();
    }
    public class moveSlidesAction implements Action {
        private boolean first = true;
        private int target_pos = 0;
        public moveSlidesAction(int position){
            target_pos = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(first){
                moveSlides(target_pos);
                first = false;
                return true;
            }
            return slides.isBusy();
        }
    }
    public Action slideAction(int position) {
        return new moveSlidesAction(position);
    }
    public class moveArmAction implements Action {
        private int endpos = 0;
        private boolean partial_motion = false;
        private boolean first = true;
        private int target_pos = 0;

        private boolean direction_up = true;
        public moveArmAction(int position){

            target_pos = position;


        }
        public moveArmAction(int position, int endpos){
            this.endpos = endpos;
            partial_motion = true;
            target_pos = position;


        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            telemetryPacket.put("Arm Target Position", target_pos);
            telemetryPacket.put("Arm Position", arm.getCurrentPosition());
            if(first){
                moveArm(target_pos);
                first = false;
                direction_up = arm.getCurrentPosition() < endpos;
                return true;
            }

            if(partial_motion){

                return direction_up ? arm.getCurrentPosition() < endpos : arm.getCurrentPosition() > endpos;
            }
            return arm.isBusy();
        }
    }
    public class updateAction implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }
    public Action updateAction(){
        return new updateAction();
    }
    public Action armAction(int position ) {
        return new moveArmAction(position);
    }
    public Action armAction(int position, int end_position) {
        return new moveArmAction(position, end_position);
    }


}
