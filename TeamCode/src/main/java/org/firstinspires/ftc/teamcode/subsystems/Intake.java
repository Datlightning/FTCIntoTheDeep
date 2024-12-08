package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;

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
import org.firstinspires.ftc.teamcode.library.BulkRead;
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
    public static PIDFCoefficients slidesPID = new PIDFCoefficients(0.007, 0.00001, 0.000003, 0.0005);
    DigitalChannel magnet_sensor;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    ElapsedTime timer;

    public TrafficLight trafficLight;

    public Distance distance;


    private boolean claw_open = false;
    private boolean wrist_open =false;

    private int level_offset=  0;

    private boolean inside_pick_up = false;
    private boolean power_four_bar_enabled = false;
    private boolean forward_delay = false;
    private boolean backward_delay = false;
    private boolean use_fast_pid = false;
    private boolean distance_update = true;
    private boolean distance_scope = false;
    public static int SLIDE_CURRENT_LIMIT = 8500;
    private double slide_stuck_time = 0;
    private boolean level_on = false;
    private double target_angle = 0;//To the y-axis
    private double target_height = 0;//To the floor


    private final double[] pickup_position_1 = {100,250};//slide, motor
    private final double[] pickup_position_2 = {800.0,220};

    private final double[] slide_distance_position_1 = {200,4.5};
    private final double[] slide_distance_position_2 = {800,10.6};
    // Adjust current threshold based on battery voltage
    public DiffyClaw diffyClaw;

    public static int distance_to_camera = 225;
    public static double feedforward_turning_point = 0;
    public static int arm_at_0_ticks = 180;
    public static int arm_at_90_ticks = 1380;
    public static double wrist_90 = 0.27;
    public static double wrist_180 = 0.62;
    public static double slide_ticks_to_inches = 0.0171875;
    public static double slide_starting_length = 13;
    public static double slide_starting_height = 9.125;
    public static double slide_width = 3.125;
    public static double claw_height = 8;
    public static int offset = 0;
    public static int offset_distance_to_0 = 0;
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
        diffyClaw = new DiffyClaw(hardwareMap, telemetry);
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        diffyClaw.mountTrafficLight(trafficLight);
        arm.setMin(-500);
        arm.setReachedRange(50);
    }
    public Intake(HardwareMap hardwareMap, Telemetry telemetry, ElapsedTime timer, TrafficLight trafficLight){
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
        diffyClaw = new DiffyClaw(hardwareMap, telemetry);
        diffyClaw.left.mountTimer(timer);
        diffyClaw.right.mountTimer(timer);
        diffyClaw.mountTrafficLight(trafficLight);
        this.trafficLight = trafficLight;

    }
    public void mountDistance(Distance distance){
        distance_update = false;
        this.distance = distance;
    }
    public void setInsidePick(boolean on){
        inside_pick_up = on;
        if(claw_open){
            openClaw();
        }else{
            closeClaw();
        }
    }
    public void toggleInsidePick(){
        setInsidePick(!inside_pick_up);
    }
    public void toggleClaw(){
        claw_open = !claw_open;
        if(claw_open){
            openClaw();
        }else{
            closeClaw();
        }
    }
    private boolean magnet_activated(){
        return !magnet_sensor.getState();
    }
    public void useFastPID(boolean on){
        use_fast_pid = on;
    }
    public void calculateOffset(){
        arm.setAbsPower(-0.1);

        double time_offset = timer.time();
        while(!magnet_activated()){
            telemetry.addData("Position",arm.getCurrentPosition());
            telemetry.update();
            if(timer.time() - time_offset > 1){
                return;
            }
        }
        arm.resetEncoder();
//        offset = arm.getCurrentPosition();
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
        arm.move_async(targetPosition + offset);
    }


    public void moveWrist(double wristAngle){
        diffyClaw.setWristAngle(wristAngle);
    }
    public void moveClaw(double clawPosition){
        diffyClaw.moveClaw(clawPosition);
    }
    public void turnClaw(double clawAngle){
        diffyClaw.setClawAngle(-clawAngle);
    }
    public void turnAndRotateClaw(double wristAngle, double clawAngle){
        moveWrist(wristAngle);
        turnClaw(clawAngle);
    }
    public void moveSlides(int targetPosition){
        slides.move_async(targetPosition);
    }
    public void setRotationPower(double power){ arm.setManualPower(power); }
    public void setTargetHeight(double target_height){ this.target_height = target_height; }
    public void setAbsRotationPower(double power){ arm.setAbsPower(power);}
    public void setSlidePower(double power){slides.setManualPower(power);}
    public void setAbsSlidePower(double power){
        slides.setAbsPower(power);
    }
    public void openClaw() {
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_open);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_open);
        }
        claw_open=true;
    }

    public void setDistanceScoping(boolean on){
        if(!on){
            trafficLight.green(false);
        }
        distance_scope = on;
    }


    public Action yeetSample(){
        //TODO make sure that it "yeets" the sample
        return new SequentialAction(
                grab(0.95),
                armAction(ARM_LIMIT),
                grab(0.74),
                armAction(0)
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
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> arm.setExitWithTime(true)),
                armAction(ARM_LIMIT-100,ARM_LIMIT - 500),
                new InstantAction(() -> moveWrist(115)),
                new ParallelAction(
                        slideAction(1250),
                        armAction(ARM_LIMIT-100)
                ),
                new InstantAction(() -> moveWrist(30)),
                new SleepAction(0.2)


        );
    }
    public Action raiseArmButNotSnagOnBasket(){
        return new SequentialAction(
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> arm.setExitWithTime(true)),
                armAction(ARM_LIMIT-100,ARM_LIMIT - 500),
                new InstantAction(() -> moveWrist(180)),
                new ParallelAction(
                        slideAction(1250),
                        armAction(ARM_LIMIT-100)
                ),
                new InstantAction(() -> moveWrist(30)),
                new SleepAction(0.2)


        );
    }
    public Action raiseArm(boolean arm_exit_with_time){
        return new SequentialAction(
                new InstantAction(() -> slides.setExitWithTime(arm_exit_with_time)),
                new InstantAction(() -> arm.setExitWithTime(arm_exit_with_time)),
                armAction(ARM_LIMIT-100,ARM_LIMIT - 500),
                new InstantAction(() -> turnAndRotateClaw(90,0)),
                new ParallelAction(
                        slideAction(1250),
                        armAction(ARM_LIMIT-100)
                ),
                new InstantAction(() -> moveWrist(30)),
                new SleepAction(0.2)

        );
    }
    public Action score(){
        return new SequentialAction(
                new InstantAction(() -> moveClaw(RobotConstants.claw_floor_pickup)),
                new SleepAction(0.1),
                new InstantAction(() -> moveWrist(90)),
                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                new InstantAction(() -> turnClaw(0)),
                slideAction(0)

        );
    }
    public Action score(boolean extra_claw_clearance){
        return new SequentialAction(
                new InstantAction(this::openClaw),
                new SleepAction(0.1),
                new InstantAction(() -> turnClaw(0)),
                new InstantAction(() -> moveWrist(extra_claw_clearance ? 100 : 90)),
                new InstantAction(() -> arm.setExitWithTime(false)),
                new InstantAction(() -> slides.setExitWithTime(false)),
                slideAction(0)

        );
    }
    public void setClawPWM(boolean on){
        if(on){
            diffyClaw.claw.enableServo();
            return;
        }
        diffyClaw.claw.disableServo();
    }
    public void setWristPWM(boolean on){
        if(on){
            diffyClaw.left.enableServo();
            diffyClaw.right.enableServo();
            return;
        }
        diffyClaw.left.disableServo();
        diffyClaw.right.disableServo();    }
    public boolean WristPWMOn(){
        return diffyClaw.left.isPWMEnabled();
    }
    public boolean ClawPWMOn(){
        return diffyClaw.claw.isPWMEnabled();
    }
    public void closeClaw() {
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_closed);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_closed);
        }
        claw_open=false;
    }
    public void plowClaw(){
        if(inside_pick_up){
            diffyClaw.moveClaw(RobotConstants.inside_pickup_open);
        }else {
            diffyClaw.moveClaw(RobotConstants.claw_open);
        }
        claw_open=true;
    }
    public boolean isInsidePick(){
        return inside_pick_up;
    }
    public void enableLevel(boolean on){
        level_on = on;
    }
    public void turnClawMore(double turnAmount){
        turnClaw(diffyClaw.getClawAngle() + turnAmount);
    }
    public void foldWrist(){
        diffyClaw.setWristAngle(RobotConstants.wrist_folded); wrist_open=false;
    }
    public void extendWrist(){
        diffyClaw.setWristAngle(RobotConstants.wrist_extended); wrist_open=true;
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
        double target_position = 90 + arm_angle + target_angle;

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

//        double arm_angle = Math.asin((target_height + claw_height - slide_starting_height) / (calculateSlideLength(slide_position)));
//        int arm_angle_in_ticks = (int) (arm_angle / Math.toRadians(90.0) * (arm_at_90_ticks - arm_at_0_ticks)) + arm_at_0_ticks + offset;

        return (int) ((pickup_position_1[1] - pickup_position_2[1])/(pickup_position_1[0] - pickup_position_2[0]) * (slide_position - pickup_position_1[0]) + pickup_position_1[1]) + (inside_pick_up ? 70 : 15);
    }
    public void slidesStuck(){
        if(slides.getCurrent() > SLIDE_CURRENT_LIMIT ){
                trafficLight.flashRed(0.5, 2);
                arm.setManualPower(0.3);
                slides.setManualPower(-0.2);

        }else{
            slides.setManualPower(0);
            arm.setManualPower(0);
        }
    }
    public void incrementLevelOffset(int a){
        level_offset += a;
        if(level_offset < 0){
            level_offset = 0;
        }
    }


    @Override
    public void update() {
        diffyClaw.update();
        trafficLight.update();
        if(distance_scope) {
            trafficLight.green(distance.getFilteredDist() < 10);
        }
        if(slides.getCurrent() > SLIDE_CURRENT_LIMIT){
            moveSlides(slides.getCurrentPosition());
            trafficLight.flashRed(0.5, 2);
        }
        if(level_on) {

            moveArm(calculateArmPosition(slides.getCurrentPosition()) + level_offset);
//           slidesStuck();
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
        if(distance_update) {
            distance.update();
        }
//        telemetry.addData("armTargetPositions", armTargetPositions.toString());
        arm.update();
        slides.update();
    }
    public int calculateSlidePositionForFloorPickup(double distance){
        double slope = (slide_distance_position_2[0] - slide_distance_position_1[0])/(slide_distance_position_2[1] - slide_distance_position_1[1]);

        return (int) (slide_distance_position_1[0] + slope * (distance - slide_distance_position_1[1]));
    }
    @Override
    public void telemetry() {
        telemetry.addData("Arm Angle", getArmAngle());
        diffyClaw.telemetry();
        slides.telemetry();
        arm.telemetry();
    }
    public void init_without_encoder_reset(){

        arm.setMaxAcceleration(5000);
        arm.setMaxDeceleration(1500);
        arm.setMaxVelocity(5000);
        arm.setUseMotionProfile(true);

        slides.setUseMotionProfile(true);
        slides.setMax(1400);
        arm.setMax(ARM_LIMIT);
        arm.setMin(-10);
        slides.setMaxAcceleration(9000);
        slides.setMaxVelocity(12000);
        slides.setMaxDeceleration(3000);
    }

    @Override
    public void init() {
        init_without_encoder_reset();
        arm.init();
        diffyClaw.init();
        slides.init();

        moveWrist(0);
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
            }

            if(partial_motion){
                if(!arm.isBusy()){
                    return false;
                }
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
