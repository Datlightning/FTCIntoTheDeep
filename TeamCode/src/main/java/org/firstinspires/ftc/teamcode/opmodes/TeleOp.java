package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.specimen_deliver;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.MultiClick;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.Rigging;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group="Actually important opmodes")
@Config
public class TeleOp extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;

//    Camera camera;
    Distance distance;

    Distance rear_distance;

    TrafficLight trafficLight;

    MultiClick multiClick;

    Rigging rigging;
    BulkRead bulkRead;
    ElapsedTime timer;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    private enum INTAKE_POSITIONS {
        START,
        LOWER_CLAW,
        TURN_ARM,
        EXTEND,
        FOLD,
        CLOSE_AND_FOLD,
        RETRACT_TO_LEAVE_2,
        FOLD_IN,
        FOLD_IN_2,
        DELIVER,
        TURN_CLAW,
        PICK_UP_FLOOR,
        GRAB_FLOOR,
        RAISE_CLAW,
        DROP_ARM,
        SCORE_SPECMEN,
        RELEASE,
        FOLD_IN_AFTER_SPECIMEN,
        RAISE_ARM_FOR_SPECIMEN,
        WALL_SPECIMEN_PICKUP,
        REVERSE,
        FORWARD,
        EXIT_FLOOR_PICKUP_SEQ,
        LOWER_SLIDES_BEFORE_FOLD, EXTEND_SLIDE_AFTER_START_ARM_RAISE, EXTEND_SLIDES_FOR_SPECIMEN, RETRACT_SLIDES_SPECI, GRAB_FLOOR_SPECIMEN, SECOND_START, DELIVER_SPECIMEN, DROP_SAMPLE, PICK_UP_FLOOR_SPECIMEN, GRAB_SPECIMEN, SWITCH_POINT, LOWER_SLIDES_AFTER_SCORING_SPECIMEN, LOWER_SLIDES_INTERMEDIATE, LOW_BASKET_EXTEND, ENABLE_DAMP, RAISE_ARM
    }
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS previous_position = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS next_position2 = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS next_position3 = INTAKE_POSITIONS.START;
    boolean move_next = false;
    boolean move_next2 = false;
    boolean move_next3 = false;

    boolean move_next_override = false;
    boolean move_next2_override = false;
    boolean move_next3_override = false;

    boolean robot_go_kaboom = false;
    boolean camera_align = false;
    int open_claw_specimen = 2;
    public static boolean player2 = true;
    public static double target_height = 8.75;
    boolean drive_pid_on = false;
    public static int offset = 0;
    public static boolean telemetry_on = false;
    double current_time = 0;
    double delay = 0;

    private boolean specimen_cycle = false;
    private boolean low_basket = false;

    int rigging_position = 0;

    private int deliver_position = ARM_LIMIT - 125;
    public static boolean player_2_on = true;
    private boolean waiting_for_distance = false;
    double distance_wait = 0;
    private boolean claw_turning = false;

    private boolean maintain_level = false;
    private boolean reset_claw = false;

    private boolean disable_distances = false;

    private final double[] previous_looptimes = new double[]{0,0,0,0,0};
    public double claw_offset = 0;

    private boolean special_testing = false;

    private double SPECIMEN_HEIGHT = 13.25;

    public void runOpMode() {
        bulkRead = new BulkRead(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        multiClick = new MultiClick(telemetry);

        distance = new Distance(hardwareMap, telemetry, RobotConstants.distance, timer);
        rear_distance = new Distance(hardwareMap, telemetry, RobotConstants.rear_distance, timer);

        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        mecaTank = new MecaTank(hardwareMap, telemetry, timer, trafficLight);
        rigging = new Rigging(hardwareMap, telemetry, timer);
//        camera = new Camera(hardwareMap, telemetry);
        mecaTank.mountFrontDistance(distance);
        mecaTank.mountRearDistance(rear_distance);
        intake.mountDistance(distance);
        if(RobotConstants.auto_transfer){
            offset = RobotConstants.offset;
            intake.init_without_encoder_reset();
            RobotConstants.auto_transfer = false;
//            mecaTank = new MecaTank(hardwareMap, telemetry, timer, RobotConstants.imu);
        }else {
            intake.init();
            sleep(200);
//            intake.calculateOffset();
            offset = intake.getOffset();
        }
//        camera.init();
//        camera.useInsidePick(intake.isInsidePick());
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        intake.setTargetAngle(90);
        intake.arm.setUseMotionProfile(true);
        intake.slides.setUseMotionProfile(true);
        mecaTank.init();
        rigging.init();
        bulkRead.clearCache();
        intake.moveSlides(intake.slides.getCurrentPosition());
        intake.moveArm(intake.arm.getCurrentPosition());
        ElapsedTime loopTimer = new ElapsedTime();
        waitForStart();
        timer.reset();
//        rigging.unlatchHooks();
//        camera.stopCamera();
        while (!isStopRequested() && opModeIsActive()) {
            loopTimer.reset();
            bulkRead.clearCache();
            distance.clearCalls();
            rear_distance.clearCalls();

            intake.setClawOffset(claw_offset);

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);


            boolean left_bumper2 = currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
            multiClick.update("left_bumper", getRuntime(), left_bumper2);

            if(multiClick.getTaps("left_bumper") == 3 || intake.magnetOnRisingEdge()){
                multiClick.clearTaps("left_bumper");
                deliver_position = ARM_LIMIT - 200;
                intake.arm.resetEncoder();
            }


            boolean right_bumper2 = currentGamepad2.right_bumper && !previousGamepad2.right_bumper;
            multiClick.update("right_bumper", getRuntime(), right_bumper2);
            if(multiClick.getTaps("right_bumper") == 3){
                multiClick.clearTaps("right_bumper");
                reset_claw = !reset_claw;
                intake.setInsidePick(false);
                if(!reset_claw){
                    trafficLight.green(false);
                    trafficLight.red(false);
                }
            }
            if(reset_claw){
                telemetry.addData("Claw Offset", claw_offset);
                intake.moveClaw(RobotConstants.claw_closed);
                trafficLight.green(false);
                trafficLight.red(true);
            }





            boolean b = (currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b);
            multiClick.update("b", getRuntime(), b);
            if(disable_distances){
                trafficLight.red(true);
            }
            boolean dpad_left2 = player2 && currentGamepad2.dpad_left && !previousGamepad2.dpad_left;
            multiClick.update("dpad_left2", getRuntime(), dpad_left2);

            if(multiClick.getTaps("dpad_left2") > 0){
                if (multiClick.getTaps("dpad_left2") == 2){
                    intake.moveWrist(90);
                    intake.moveArm(1500);
                }else {
                    switch (rigging_position) {
                        case 0:
                            rigging.raiseHooks();
                            rigging_position += 1;
                            break;
                        case 1:
                            rigging.rig();
                            rigging_position = 0;
                            break;
                    }
                }
                multiClick.clearTaps("dpad_left2");
            }
            if(multiClick.getTaps("b") == 3){
                multiClick.clearTaps("b");
                robot_go_kaboom = !robot_go_kaboom;
                if(!robot_go_kaboom) {
                    intake.init();
                    intake.moveWrist(0);
                    intake.moveArm(0);
                    intake.moveSlides(0);
                    trafficLight.red(false);
                }else{
                    intake.moveWrist(0);
                    trafficLight.flashRed(0.4, 3);
                }
            }

            if (b) {
                multiClick.clearTaps("b");
                intake.moveSlides(intake.slides.getCurrentPosition());
                intake.moveArm(intake.arm.getCurrentPosition());
                rigging.rigging_motor.move_async(rigging.rigging_motor.getCurrentPosition());
                mecaTank.forceExit();
                move_next3 = false;
                move_next2 = false;
                move_next = false;
                maintain_level = false;
                intake.setDamp(false);
                intake.specimen_level_on = false;
                trafficLight.flashOffred(1, 1);
                trafficLight.red(false);
                trafficLight.green(false);
                trafficLight.orange(false);
                intake.enableLevel(false);


            }
            if(intake.specimen_level_on){
                intake.setTargetHeight(intake.getSpecimenHeight());
            }
//            player2 = !currentGamepad1.dpad_left && player_2_on; //goodbye player 2 button o7

            //claw rotations
            if(position.equals(INTAKE_POSITIONS.SCORE_SPECMEN)){
                if (currentGamepad2.right_trigger != 0){
                    intake.moveWrist(specimen_deliver - currentGamepad2.right_trigger * 30);
                }
                else if(currentGamepad2.left_trigger != 0){
                    intake.moveWrist(specimen_deliver - currentGamepad2.left_trigger * 30);
                }
            }else {
                if (currentGamepad2.left_trigger != 0 && intake.slides.getCurrentPosition() <= 900) {
                    intake.turnClaw(currentGamepad2.left_trigger * -90);
                    claw_turning = true;
                    camera_align = false;
                } else if (currentGamepad2.right_trigger != 0 && intake.slides.getCurrentPosition() <= 900) {
                    intake.turnClaw(currentGamepad2.right_trigger * 90);
                    claw_turning = true;
                    camera_align = false;
                } else if (claw_turning && !camera_align) {
                    claw_turning = false;
                    intake.turnClaw(0);
                }
            }

            //emergency claw reset functionality and normal claw movement
            boolean dpad_right =  (player2 &&  currentGamepad2.dpad_right && !previousGamepad2.dpad_right) || (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
            boolean dpad_left =  (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);
            multiClick.update("dpad_right", getRuntime(), dpad_right);
            multiClick.update("dpad_left", getRuntime(), dpad_left);
            if(multiClick.getTaps("dpad_left") == 1 && reset_claw){
                claw_offset -= 0.01;
                multiClick.clearTaps("dpad_left");
            }
            if(multiClick.getTaps("dpad_left") == 2 && reset_claw){
                claw_offset -= 0.05;
                multiClick.clearTaps("dpad_left");
            }
            if(multiClick.getTaps("dpad_left") == 1 && !reset_claw){
                disable_distances = !disable_distances;
                multiClick.clearTaps("dpad_left");

            }else if(multiClick.getTaps("dpad_left") == 2 && !reset_claw){
                intake.setNewTargetHeight();
            }
            if(multiClick.getTaps("dpad_right") == 1){
                if(reset_claw){
                    claw_offset += 0.01;
                }else {
                    intake.toggleClaw();
                }
                multiClick.clearTaps("dpad_right");
            }else if(multiClick.getTaps("dpad_right") > 1){
                if(reset_claw){
                    claw_offset += 0.05;
                }else {
                    intake.toggleInsidePick();
                }
//                camera.useInsidePick(intake.isInsidePick());
                multiClick.clearTaps("dpad_right");
            }

            //all the drive controls
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

            multiClick.update("a", getRuntime(),  ( currentGamepad1.a && !previousGamepad1.a));
            multiClick.update("a2", getRuntime(), (player2  && currentGamepad2.a && !previousGamepad2.a));

            if(currentGamepad1.back && !previousGamepad1.back){
                specimen_cycle = !specimen_cycle;
            }
            multiClick.update("back",  getRuntime(), currentGamepad1.back && !previousGamepad1.back);
            if (multiClick.getTaps("back") != 0){
                switch (multiClick.getTaps("back")){
                    case 2:
                        specimen_cycle = true;
                        low_basket = false;
                        break;
                    case 3:
                        specimen_cycle = false;
                        low_basket = true;
                        break;
                    case 1:
                    default:
                        specimen_cycle = false;
                        low_basket = false;
                }
                multiClick.clearTaps("back");
            }

            if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
                if (intake.slides.getCurrentPosition() > 900){
                    trafficLight.red(true);
                }else{
                    trafficLight.red(false);
                    trafficLight.green(false);
                }
            }
            //"basic" arm controls
            if (move_next || multiClick.getTaps("a") > 0 || multiClick.getTaps("a2") > 0) {
                trafficLight.flashGreen(0.5, multiClick.getTaps("a"));
                // Move forward if single tap, move backward if double tap
                if (multiClick.getTaps("a") == 1 || move_next || multiClick.getTaps("a2") == 1) {
                    position = next_position;  // Move forward
                } else if (multiClick.getTaps("a") == 2 || multiClick.getTaps("a2") == 3) {
                    position = previous_position;  // Move backward
                }
                move_next3 = false;
                move_next2 = false;
                if(intake.specimen_level_on){
                    intake.enableLevel(false);
                }
                intake.specimen_level_on = false;
                move_next_override = (multiClick.getTaps("a") > 0 || multiClick.getTaps("a2") > 0);
                multiClick.clearTaps("a2");
                multiClick.clearTaps("a");
                switch (position) {
                    case START:
                        move_next = true;
                        if(intake.slides.getCurrentPosition() > 200) {
                            intake.moveSlides(0);
                        }
                        next_position = INTAKE_POSITIONS.SECOND_START;
                        previous_position = INTAKE_POSITIONS.START; // Set previous position for double tap
                        break;
                    case SECOND_START:
                        if(intake.slides.isBusy() && !move_next_override){
                            break;
                        }
                        move_next = true;
                        intake.plowClaw();
                        intake.moveWrist(90);
                        intake.slides.setMax(650);
                        intake.setFourBar(false);
                        intake.setTargetHeight(12);
                        intake.arm.setMax(350);
                        intake.enableLevel(true);
                        maintain_level = true;
                        next_position = INTAKE_POSITIONS.EXTEND_SLIDE_AFTER_START_ARM_RAISE;
                        previous_position = INTAKE_POSITIONS.START; // Set previous position for double tap
                        break;
                    case EXTEND_SLIDE_AFTER_START_ARM_RAISE:
                        if(intake.arm.isBusy() && !move_next_override){
                            break;
                        }
                        move_next = false;
                        intake.moveSlides(Math.max(150, intake.slides.getCurrentPosition()));
                        next_position = INTAKE_POSITIONS.LOWER_CLAW;
                        previous_position = INTAKE_POSITIONS.SECOND_START;
                        break;
                    case LOWER_CLAW:
                        intake.plowClaw();
                        intake.setTargetAngle(90);
                        intake.setTargetHeight(12);
                        intake.enableLevel(true);
                        intake.setFourBar(true);
                        intake.arm.setMax(350);
                        intake.slides.setMax(1000);
                        intake.arm.setUseMotionProfile(false);
                        intake.slides.setUseMotionProfile(false);
                        maintain_level = true;
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;//REMOVED DROP ARM
                        previous_position = INTAKE_POSITIONS.SECOND_START;
                        break;
                    case DROP_ARM:
                        intake.useFastPID(true);
                        intake.enableLevel(false);
                        maintain_level = false;
                        intake.setFourBar(true);
                        intake.moveArmUntilZeroSpeed(-0.4);
                        delay = 0;
                        current_time = timer.time();
                        move_next = true;
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;
                    case CLOSE_AND_FOLD:
                        if (intake.getVeloOn() && !move_next_override) {
                            break;
                        }
                        trafficLight.red(false);
                        intake.useVelo(false);
                        move_next = true;
                        intake.setFourBar(false);
                        mecaTank.setMaxPower(1);
                        current_time = timer.seconds();
                        intake.closeClaw(-0.03);
                        intake.slides.setMax(700);
                        intake.enableLevel(false);
                        intake.arm.setMax(Math.max(ARM_LIMIT, deliver_position));
                        current_time = timer.time();
                        delay = 0.3;
                        next_position = INTAKE_POSITIONS.RAISE_ARM;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;
                    case DELIVER_SPECIMEN:
                        intake.arm.setMax(Math.max(ARM_LIMIT, deliver_position));
                        intake.moveSlides(700);
                        intake.moveWrist(90);
                        next_position = INTAKE_POSITIONS.DROP_SAMPLE;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        move_next = false;
                        break;
                    case DROP_SAMPLE:
                        intake.openClaw();

                        next_position = INTAKE_POSITIONS.START;
                        previous_position = INTAKE_POSITIONS.DELIVER_SPECIMEN;
                        move_next = true;
                        break;
                    case RAISE_ARM:
                        if(timer.time() - current_time < delay && !move_next_override){
                            break;
                        }
                        move_next = false;
                        intake.arm.setMax(Math.max(ARM_LIMIT, deliver_position));
                        intake.slides.setMax(700);
                        current_time = timer.seconds();
                        intake.enableLevel(false);
//                        intake.moveWrist(180);
                        intake.useFastPID(true);
                        intake.slides.setUseMotionProfile(true);
                        intake.moveArm(350);
                        if(intake.slides.getCurrentPosition() > 600){
                            intake.moveSlides(600);
                        }
                        intake.setFourBar(false);
                        next_position = INTAKE_POSITIONS.FOLD;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;

                    case FOLD:
                        if(intake.slides.isBusy() && !move_next_override){
                            break;
                        }
                        intake.arm.setMax(Math.max(ARM_LIMIT, deliver_position));

                        intake.moveArm(300);
                        move_next = false;
                        intake.arm.setUseMotionProfile(true);
                        intake.slides.setUseMotionProfile(true);
                        intake.useFastPID(false);
                        intake.moveWrist(90);
                        intake.moveSlides(0);
//                        camera.stopCamera();
                        if(!specimen_cycle){
                            next_position = INTAKE_POSITIONS.TURN_ARM;
                            previous_position = INTAKE_POSITIONS.RAISE_ARM;
                        }else{
                            next_position = INTAKE_POSITIONS.DELIVER_SPECIMEN;
                            previous_position = INTAKE_POSITIONS.RAISE_ARM;
                        }

                        break;
                    case LOWER_SLIDES_BEFORE_FOLD:
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.FOLD;
                        mecaTank.setMaxPower(1);
                        move_next = false;
                        break;
                    case TURN_ARM:
                        intake.arm.setMax(Math.max(ARM_LIMIT, deliver_position));

                        intake.moveArm(deliver_position);
                        intake.slides.setMax(1400);
                        intake.moveWrist(120);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.LOW_BASKET_EXTEND;
                        previous_position = INTAKE_POSITIONS.FOLD;
                        break;
                    case LOW_BASKET_EXTEND:
                        if (intake.arm.getCurrentPosition() < 300 && !move_next_override) {
                            break;
                        }
                        intake.slides.setMax(1400);
                        intake.moveSlides(400);
                        if(low_basket){
                            move_next = true;
                            next_position = INTAKE_POSITIONS.TURN_CLAW;
                        }else{
                            move_next = false;
                            next_position = INTAKE_POSITIONS.EXTEND;
                        }
                        previous_position = INTAKE_POSITIONS.FOLD;

                        break;

                    case EXTEND:
                        if (intake.arm.getCurrentPosition() < 1000 && !move_next_override) {
                            break;
                        }
                        intake.slides.setMax(1400);
                        intake.slides.setUseMotionProfile(true);
                        intake.moveSlides(1350);
                        mecaTank.setMaxPower(0.5);
                        current_time = timer.time();
                        move_next = true;
                        delay = 0.7;
                        next_position = INTAKE_POSITIONS.TURN_CLAW;
                        previous_position = INTAKE_POSITIONS.LOWER_SLIDES_BEFORE_FOLD;
                        break;

                    case TURN_CLAW:
                        if (intake.slides.getCurrentPosition() < (low_basket ? 0 : 1250) && !move_next_override) {
                            break;
                        }
                        move_next = false;
                        intake.moveWrist(30);
                        next_position = INTAKE_POSITIONS.DELIVER;
                        previous_position = INTAKE_POSITIONS.FOLD;
                        break;

                    case DELIVER:
                        mecaTank.setMaxPower(1);
                        intake.openClaw();
                        delay = 0.3;
                        current_time = timer.time();
                        if (intake.arm.getCurrentPosition() - deliver_position > 0 ){

                            deliver_position = intake.arm.getCurrentPosition();
                            if(deliver_position > intake.arm.maxHardstop){
                                intake.arm.setMax(deliver_position);
                            }
                        }
                        move_next = true;
                        next_position = INTAKE_POSITIONS.RAISE_CLAW;
                        previous_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;

                    case RAISE_CLAW:
                        if (timer.time() - current_time < delay && !move_next_override) {
                            break;
                        }
                        current_time = timer.time();
                        delay = 0.1;
                        intake.moveWrist(105);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE_2;
                        previous_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;

                    case RETRACT_TO_LEAVE_2:
                        if (timer.time() - current_time < delay && !move_next_override) {
                            break;
                        }
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.FOLD_IN_2;
                        previous_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;

                    case FOLD_IN_2:
                        if (intake.slides.getCurrentPosition() < 300 || move_next_override) {
                            intake.foldWrist();
                            intake.moveArm(200);
                            next_position = INTAKE_POSITIONS.SECOND_START;
                            previous_position = INTAKE_POSITIONS.TURN_CLAW;
                            move_next = false;
                        }
                        break;
                    default:
                        next_position = INTAKE_POSITIONS.SECOND_START;
                        move_next = true;
                        break;
                }
            }


            //automatic distance pickup
            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR) || position.equals(INTAKE_POSITIONS.GRAB_FLOOR_SPECIMEN)){
//                distance.setOn(!disable_distances);

                if(!disable_distances && distance.getFilteredDist() > RobotConstants.TOO_CLOSE && distance.getFilteredDist() < RobotConstants.TOO_FAR){
                    if (waiting_for_distance){
                        if(timer.time() - distance_wait > 0.3) {
                            intake.closeClaw();
                            distance.setOn(false);
                        }

                    }else {
                        waiting_for_distance = true;
                        distance_wait = timer.time();
                    }

                }else{
                    waiting_for_distance = false;
                }
            }
                if(telemetry_on){
                    telemetry.addData("Waiting For Distance", waiting_for_distance);
                    telemetry.addData("Distance Waiting Time", timer.time() - distance_wait);
                    telemetry.addData("Distance Wait", distance_wait);
                }

            //floor pickup
            boolean x = (player2 && currentGamepad2.x && !previousGamepad2.x) || ( currentGamepad1.x && !previousGamepad1.x);
            multiClick.update("x", getRuntime(), x);
            if (multiClick.getTaps("x") > 0 || move_next2) {
                move_next2_override = multiClick.getTaps("x") > 0;

                trafficLight.flashGreen(0.5, multiClick.getTaps("x"));

                if(multiClick.getTaps("x") > 1){
                    move_next2 = true;
                    position = INTAKE_POSITIONS.PICK_UP_FLOOR;

                }
                else {
                    intake.enableLevel(false);
                    intake.setFourBar(false);
                    intake.setInsidePick(false);
                    intake.specimen_level_on = false;
//                    camera.useInsidePick(false);
                    move_next3 = false;
                    move_next = false;

                    mecaTank.setMaxPower(1);
                    intake.slides.setMax(1400);

                    switch (position) {
                        case PICK_UP_FLOOR:
                            if(intake.slides.isBusy() && !move_next2_override){
                                break;
                            }
                            intake.moveClaw(RobotConstants.claw_fully_open);
                            distance.setOn(!disable_distances);
                            intake.setDistanceScoping(true);
                            intake.moveArm(0);
                            next_position2 = INTAKE_POSITIONS.GRAB_FLOOR;
                            move_next2 = false;
                            break;
                        case GRAB_FLOOR:
                            if (!mecaTank.isBusy() || move_next2_override) {
                                intake.setDistanceScoping(false);
                                distance.setOn(false);
                                intake.closeClaw();
                                delay = 0.3;
                                current_time = timer.time();
                                next_position2 = INTAKE_POSITIONS.EXIT_FLOOR_PICKUP_SEQ;
                                move_next2 = false;

                            }
                            break;
                        case EXIT_FLOOR_PICKUP_SEQ:
                            if(timer.time() - current_time > delay || move_next2_override){
                                move_next = true;
                                move_next2 = false;
                                next_position = INTAKE_POSITIONS.FOLD;
                            }
                            break;

                        default:
                            intake.moveSlides(0);
                            intake.moveWrist(RobotConstants.floor_pickup_position);
                            intake.moveClaw(RobotConstants.claw_fully_open);
                            next_position2 = INTAKE_POSITIONS.PICK_UP_FLOOR;
                            move_next2 = true;
                            break;
                    }
                    position = next_position2;
                }
                multiClick.clearTaps("x");
            }

            //manual slide and arm controls
            boolean left_bumper = currentGamepad1.left_bumper;
            boolean right_bumper = currentGamepad1.right_bumper;

            if (left_bumper) {
                intake.setSlidePower(-0.4);
            } else if (right_bumper) {
                intake.setSlidePower(0.4);
            }else if(currentGamepad2.right_stick_y != 0 && !reset_claw){
                double power = sameSignSqrt(-currentGamepad2.right_stick_y);
                if(!robot_go_kaboom) {
                    intake.setSlidePower(power);
                }else{
                    intake.slides.setAbsPower(power);
                }

            }
            else if (!robot_go_kaboom){
                intake.setSlidePower(0);
            }else{
                intake.slides.setAbsPower(0);
            }
            boolean dpad_down = currentGamepad1.dpad_down;
            boolean dpad_up = currentGamepad1.dpad_up;
            if(dpad_down && !previousGamepad1.dpad_down && intake.getLevelOffset() <= 0 && maintain_level){
                intake.absIncrementLevelOffset(-30);
            }
            if (dpad_down) {
                if(maintain_level){
                    intake.incrementLevelOffset(-10);
                    intake.setRotationPower(-0.25);
                }else {
                    intake.setRotationPower(-0.4);
                }
            } else if (dpad_up) {
                if(maintain_level){
                    intake.incrementLevelOffset(10);
                    intake.setRotationPower(0.25);
                }
                else{
                    intake.setRotationPower(0.4);
                }
            }
            else if(currentGamepad2.left_stick_y != 0){
                double power = sameSignSqrt((-currentGamepad2.left_stick_y/(position.equals(INTAKE_POSITIONS.TURN_CLAW) ? 5.0 : 1.0)));
                if(!robot_go_kaboom) {
                    intake.setRotationPower(power);
                }else{
                    intake.arm.setAbsPower(power);
                }
            }
            else if(!robot_go_kaboom) {
                    intake.setRotationPower(0);
            }else{
                intake.arm.setAbsPower(0);
            }


            //specimen
            boolean y = (player2 && currentGamepad2.y && !previousGamepad2.y) || (currentGamepad1.y && !previousGamepad1.y);
            multiClick.update("y", getRuntime(), y);
            if(open_claw_specimen < 2 && special_testing){
                rear_distance.setOn(!disable_distances);
                if (!disable_distances) {
                    switch (open_claw_specimen) {
                        case 0:
                            if (rear_distance.getFilteredDist() < 4) {
                                open_claw_specimen += 1;
                            }
                            break;
                        case 1:
                            if (rear_distance.getFilteredDist() > 8) {
                                intake.openClaw();
                                open_claw_specimen += 1;
                                //make one if not work
                                //last code change
                                rear_distance.setOn(false);
                            }
                            break;
                    }
                }

            }
            if(!mecaTank.isBusy() && drive_pid_on){
                drive_pid_on = false;
                distance.setOn(false);
                rear_distance.setOn(false);
            }
            if (multiClick.getTaps("y") > 0 || move_next3) {
                move_next3_override = multiClick.getTaps("y") > 0;
                move_next = false;
                move_next2 = false;
                trafficLight.flashGreen(0.5, multiClick.getTaps("y"));
                if(multiClick.getTaps("y") == 2){
                   position = INTAKE_POSITIONS.START;
                   move_next3 = true;
                }
                else {
                    mecaTank.setMaxPower(1);
                    intake.slides.setMax(1400);
/**
                    switch (position) {
                        case PICK_UP_FLOOR_SPECIMEN:
                            if(intake.slides.isBusy() || move_next3_override){
                                break;
                            }
                            intake.setDistanceScoping(true);
                            mecaTank.setDistanceType(true);
                            distance.setOn(!disable_distances);
                            intake.moveArm(0);
                            next_position3 = INTAKE_POSITIONS.GRAB_FLOOR_SPECIMEN;
                            move_next3 = false;
                            intake.moveWrist(RobotConstants.floor_pickup_position);
                            intake.moveClaw(RobotConstants.claw_fully_open);

                            break;
                        case GRAB_FLOOR_SPECIMEN:

                            if (mecaTank.isBusy() && !move_next3_override) {
                                break;
                            }
                            intake.setDistanceScoping(false);
                            distance.setOn(false);
                            intake.closeClaw();
                            delay = 0.3;
                            current_time = timer.time();
                            next_position3 = INTAKE_POSITIONS.RAISE_ARM_FOR_SPECIMEN;
                            move_next3 = false;
                            break;
                        case RAISE_ARM_FOR_SPECIMEN:
                            move_next3 = false;
                            intake.moveArm(ARM_LIMIT);
                            intake.moveSlides(200);
                            intake.moveWrist(RobotConstants.specimen_deliver);

                            next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;//change this to RELEASE if the specimen thing is not working for whatever reason
                            break;
//                        case FORWA    RD:
//                            if ((intake.arm.isBusy() || intake.slides.isBusy()) && !move_next3_override) break;
//
//                            next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
//                            break;
                        case SCORE_SPECMEN:
                            if (mecaTank.isBusy() && !move_next3_override) {
                                break;
                            }
                            intake.moveSlides(0);
                            open_claw_specimen = 0;
                            intake.moveArm(ARM_LIMIT + 50);
                            next_position3 = INTAKE_POSITIONS.RELEASE;

                            move_next3 = false;
                            break;
//                        case REVERSE:
//                            if (intake.slides.isBusy() && !move_next3_override) {
//                                break;
//                            }
//                            mecaTank.setDistanceType(false);
//                            rear_distance.setOn(!disable_distances);
//                            if(!disable_distances) {
//                                mecaTank.DrivePastDistance(10, 0.4);
//                            }
//                            drive_pid_on = true;
//                            next_position3 = INTAKE_POSITIONS.RELEASE;
//                            break;
                        case RELEASE:
                            if (mecaTank.isBusy() && !move_next3_override) {
                                break;
                            }
                            rear_distance.setOn(false);
                            open_claw_specimen = 2;

                            mecaTank.setDistanceType(true);
                            intake.openClaw();
                            move_next3 = true;
                            current_time = timer.time();
                            next_position3 = INTAKE_POSITIONS.FOLD_IN_AFTER_SPECIMEN;
                            break;
                        case FOLD_IN_AFTER_SPECIMEN:
                            if(timer.time() - current_time <= delay && !move_next3_override){
                                break;
                            }
                            intake.moveArm(0);
                            intake.moveWrist(0);
                            next_position = INTAKE_POSITIONS.START;
                            next_position3 = INTAKE_POSITIONS.START;
                            break;
                        default:
                            intake.moveSlides(0);
                            intake.moveWrist(RobotConstants.floor_pickup_position);
                            intake.moveClaw(RobotConstants.claw_fully_open);
                            move_next3 = true;
                            next_position3 = INTAKE_POSITIONS.PICK_UP_FLOOR_SPECIMEN;
                            break;
                    }
 */
                    switch (position){
                        case GRAB_SPECIMEN:

                            intake.closeClaw(-0.03);
                            intake.setFourBar(false);
                            intake.enableLevel(false);

                            intake.specimen_level_on = false;
                            move_next3 = false;
                            next_position3 = INTAKE_POSITIONS.RAISE_ARM_FOR_SPECIMEN;
                            break;
                        case RAISE_ARM_FOR_SPECIMEN:
                            intake.moveArm(1500);
                            intake.closeClaw();
                            move_next3 = true;
                            next_position3 = INTAKE_POSITIONS.EXTEND_SLIDES_FOR_SPECIMEN;
                            break;
                        case EXTEND_SLIDES_FOR_SPECIMEN:
                            if(intake.arm.getCurrentPosition() < 500 && !move_next3_override){
                                break;
                            }
                            intake.moveWrist(0);
                            intake.moveSlides(325);
                            move_next3 = false;
                            next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
                            break;
                        case SCORE_SPECMEN:
                            intake.moveSlides(800);
                            next_position3 = INTAKE_POSITIONS.SWITCH_POINT;
                            move_next3 = true;
                            break;
                        case SWITCH_POINT:
                            if(move_next3_override){
                                next_position3 = INTAKE_POSITIONS.RELEASE;
                                move_next3 = true;
                                break;
                            }
                            if (intake.isSlideStoppedWithCurrent()){
                                next_position3 = INTAKE_POSITIONS.RELEASE;
                                intake.clearStoppedWithCurrent();
                            }else if(!intake.slides.isBusy()){
                                next_position3 = INTAKE_POSITIONS.EXTEND_SLIDES_FOR_SPECIMEN;
                            }

                            move_next3 = true;
                            break;
                        case RELEASE:
                            if(intake.slides.isBusy() && !move_next3_override){
                                break;
                            }
                            move_next3 = true;
                            current_time = timer.seconds();
                            delay = 0.3;
                            intake.openClaw();
                            next_position3 = INTAKE_POSITIONS.LOWER_SLIDES_AFTER_SCORING_SPECIMEN;
                            break;
                        case LOWER_SLIDES_AFTER_SCORING_SPECIMEN:
                            if(timer.seconds() - current_time <= delay && !move_next3_override){
                                break;
                            }
                            move_next3 = false;
                            intake.moveSlides(500);
                            next_position = INTAKE_POSITIONS.START;
                            next_position3 = INTAKE_POSITIONS.START;
                            break;
                        case ENABLE_DAMP:
                            if(intake.arm.isBusy() && !move_next3_override){
                                break;
                            }
                            intake.specimen_level_on = true;
                            move_next3 = false;
                            next_position3 = INTAKE_POSITIONS.GRAB_SPECIMEN;
                            intake.setDamp(true);
                            break;
                        case LOWER_SLIDES_INTERMEDIATE:
                            if(intake.slides.getCurrentPosition() > 100 && !move_next3_override){
                                break;
                            }
                            move_next3 = true;
                            intake.setTargetHeight(intake.getSpecimenHeight());
                            intake.enableLevel(true);
                            intake.setDamp(false);
                            intake.setTargetAngle(-10);
                            intake.setFourBar(true);
                            next_position3 = INTAKE_POSITIONS.ENABLE_DAMP;
                            break;
                        default:
                            move_next3 = true;
                            intake.moveSlides(0);
                            intake.moveClaw(RobotConstants.claw_flat);
                            next_position3 = INTAKE_POSITIONS.LOWER_SLIDES_INTERMEDIATE;
                            break;

                    }
                    position = next_position3;
                }
                multiClick.clearTaps("y");
            }

            //rigging
            if (gamepad2.dpad_up){
                rigging.setManualPower(-1);
            }else if(gamepad2.dpad_down){
                rigging.setManualPower(1);
            }else{
                rigging.rigging_motor.setManualPower(0);
            }




            if(!robot_go_kaboom) {
                intake.update();
            }

            mecaTank.update();
            distance.update();
            rear_distance.update();
            rigging.update();
            addLoopTime(loopTimer.milliseconds());
            if(getAvgLoopTime() > 125 && !disable_distances && timer.seconds() > 5){
                disable_distances = true;
                mecaTank.forceExit();
            }
            if (distance.timeout()){
                telemetry.addLine("Distance Timed Out");
            }
            if (rear_distance.timeout()){
                telemetry.addLine("Rear Distance Timed Out");


            }
            if(disable_distances){
                rear_distance.setOn(false);
                distance.setOn(false);
            }
            if(telemetry_on) {
                mecaTank.telemetry();
                intake.telemetry();
                rear_distance.telemetry();
                distance.telemetry();
                rigging.telemetry();
                telemetry.addData("Front Distance On", distance.isOn());
                telemetry.addData("Rear Distance On", rear_distance.isOn());
                telemetry.addData("Rear Distance Count", rear_distance.getCalls());
                telemetry.addData("Distance Count", distance.getCalls());
                telemetry.addData("Specimen Cycle", specimen_cycle);
                telemetry.addData("Power Damp", intake.arm.power_damp);


//                camera.telemetry();
            }
            multiClick.enableTelemetry(telemetry_on);

            if (rear_distance.isOn()) {

                telemetry.addData("Rear Distance Reading", rear_distance.getFilteredDist());
            }
            telemetry.addData("Intake Position", position);
            telemetry.addData("Kaboom Mode", robot_go_kaboom);
//            telemetry.addData("Open Claw Specimen", open_claw_specimen);

            telemetry.addData("Distance Disabled", disable_distances);
            telemetry.addData("Cycle Time", loopTimer.milliseconds());
            telemetry.addData("Average Cycle Time", getAvgLoopTime());
            telemetry.addData("Arm Position", intake.arm.getCurrentPosition());
            telemetry.addData("Slide Position", intake.slides.getCurrentPosition());
            telemetry.addData("Delivery Position", deliver_position);
            telemetry.addData("Specimen Height", intake.getSpecimenHeight());

            telemetry.update();

        }
    }

    private double sameSignSqrt(float v) {
        return Math.copySign(Math.sqrt(Math.abs(v)), v);
    }
    private double sameSignSqrt(double v) {
        return Math.copySign(Math.sqrt(Math.abs(v)), v);
    }
    private void addLoopTime(double time){
        previous_looptimes[4] = previous_looptimes[3];
        previous_looptimes[3] = previous_looptimes[2];
        previous_looptimes[2] = previous_looptimes[1];
        previous_looptimes[1] = previous_looptimes[0];
        previous_looptimes[0] = time;
    }
    private double getAvgLoopTime(){
        double sum = previous_looptimes[0] + previous_looptimes[1] + previous_looptimes[2] + previous_looptimes[3] + previous_looptimes[4];
        sum /= 5;
        return sum;
    }

}

