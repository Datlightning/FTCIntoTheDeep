package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotConstants.ARM_LIMIT;
import static org.firstinspires.ftc.teamcode.RobotConstants.right_servo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.BulkRead;
import org.firstinspires.ftc.teamcode.library.MultiClick;
import org.firstinspires.ftc.teamcode.subsystems.Distance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

//TODO: Make the PID with distance great again tm
//TODO: Make the camera move slides to auto align maybe
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;

//    Camera camera;
    Distance distance;

    Distance rear_distance;

    TrafficLight trafficLight;

    MultiClick multiClick;
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
        GRAB_FLOOR_SPECIMEN,
        REVERSE,
        FORWARD,
        EXIT_FLOOR_PICKUP_SEQ,
        LOWER_SLIDES_BEFORE_FOLD, EXTEND_SLIDE_AFTER_START_ARM_RAISE, RAISE_ARM
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

    boolean camera_align = false;


    public static boolean player2 = true;
    public static double target_height = 3;
    boolean drive_pid_on = false;
    public static int offset = 0;
    public static boolean telemetry_on = false;
    double current_time = 0;
    double delay = 0;
    public static boolean player_2_on = true;
    private boolean waiting_for_distance = false;
    double distance_wait = 0;
    private boolean claw_turning = false;

    private boolean maintain_level = false;
    private boolean reset_claw = false;

    private boolean disable_distances = false;


    public double claw_offset = 0;
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
        intake.moveSlides(intake.slides.getCurrentPosition());
        intake.moveArm(intake.arm.getCurrentPosition());
        ElapsedTime loopTimer = new ElapsedTime();
        waitForStart();
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

            intake.setTargetHeight(target_height);

            boolean left_bumper2 = currentGamepad2.left_bumper && !previousGamepad2.left_bumper;
            multiClick.update("left_bumper", getRuntime(), left_bumper2);

            if(multiClick.getTaps("left_bumper") == 3){
                multiClick.clearTaps("left_bumper");
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
//                    RobotConstants.updateClawClosed(intake.diffyClaw.claw.getPosition());
                }
            }
            if(reset_claw){
                telemetry.addData("Claw Offset", claw_offset);
                intake.moveClaw(RobotConstants.claw_closed);
                trafficLight.green(false);
                trafficLight.red(true);
            }
//            if(multiClick.getTaps("right_bumper") == 1){
//                multiClick.clearTaps("right_bumper");
//                if(camera.isCameraOn()){
//                    intake.turnClaw(camera.getYellow()[0]);
//                    intake.moveSlides(intake.slides.getCurrentPosition() - 225);
//                    camera_align = true;
//                }
//            }



            boolean b = (currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b);
            multiClick.update("b", getRuntime(), b);
            if(multiClick.getTaps("b") == 2){
                disable_distances = !disable_distances;
                if(!disable_distances){
                    trafficLight.red(false);
                }
                multiClick.clearTaps("b");
            }
            if(disable_distances){
                trafficLight.red(true);
            }

            if (b) {
                multiClick.clearTaps("b");
                intake.moveSlides(intake.slides.getCurrentPosition());
                intake.moveArm(intake.arm.getCurrentPosition());
                mecaTank.forceExit();
                move_next3 = false;
                move_next2 = false;
                move_next = false;
                trafficLight.flashOffred(1, 1);
                trafficLight.red(false);
                trafficLight.green(false);
                trafficLight.orange(false);
                intake.enableLevel(false);


            }

//            player2 = !currentGamepad1.dpad_left && player_2_on; //goodbye player 2 button o7

            //claw rotations
            if(currentGamepad2.left_trigger != 0){
                intake.turnClaw(currentGamepad2.left_trigger * -90);
                claw_turning = true;
                camera_align = false;
            }else if(currentGamepad2.right_trigger != 0){
                intake.turnClaw(currentGamepad2.right_trigger * 90);
                claw_turning = true;
                camera_align = false;
            }else if(claw_turning && !camera_align){
                claw_turning = false;
                intake.turnClaw(0);
            }

            //emergency claw reset functionality and normal claw movement
            boolean dpad_right =  (player2 &&  currentGamepad2.dpad_right && !previousGamepad2.dpad_right) || (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
            boolean dpad_left =  (player2 &&  currentGamepad2.dpad_left && !previousGamepad2.dpad_left) || (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);
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
            if(dpad_left && !reset_claw){
                disable_distances = !disable_distances;
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
                move_next_override = move_next && (multiClick.getTaps("a") > 0 || multiClick.getTaps("a2") > 0);
                multiClick.clearTaps("a2");
                multiClick.clearTaps("a");
                switch (position) {
                    case START:
                        move_next = true;
                        intake.plowClaw();
                        intake.moveWrist(90);
                        intake.moveArm(180 + (intake.isInsidePick() ? 70 : 30));
                        intake.slides.setMax(1000);
                        intake.setFourBar(false);
                        intake.enableLevel(false);
                        maintain_level = false;

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
                        previous_position = INTAKE_POSITIONS.START;
                        break;
                    case LOWER_CLAW:
                        intake.moveWrist(135);
                        intake.plowClaw();
                        intake.setTargetAngle(90);
                        intake.setFourBar(true);
                        intake.arm.setUseMotionProfile(false);
                        intake.slides.setUseMotionProfile(false);
                        intake.enableLevel(true);
                        maintain_level = true;
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;//REMOVED DROP ARM
                        previous_position = INTAKE_POSITIONS.START;
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
                        intake.useVelo(false);
                        move_next = true;
                        intake.setFourBar(false);

                        mecaTank.setMaxPower(1);
                        current_time = timer.seconds();
                        intake.closeClaw();
                        next_position = INTAKE_POSITIONS.RAISE_ARM;
                        intake.slides.setMax(1400);
                        intake.enableLevel(false);
                        current_time = timer.time();
                        delay = 0.3;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;

                    case RAISE_ARM:
                        if(timer.time() - current_time < delay && !move_next_override){
                            break;
                        }
                        move_next = false;
                        intake.slides.setMax(1400);
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
                        intake.moveArm(300);
                        move_next = false;
                        intake.arm.setUseMotionProfile(true);
                        intake.slides.setUseMotionProfile(true);
                        intake.useFastPID(false);
                        intake.moveWrist(90);
                        intake.moveSlides(0);
//                        camera.stopCamera();
                        next_position = INTAKE_POSITIONS.TURN_ARM;
                        previous_position = INTAKE_POSITIONS.RAISE_ARM;
                        break;
                    case LOWER_SLIDES_BEFORE_FOLD:
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.FOLD;
                        mecaTank.setMaxPower(1);
                        move_next = true;
                        break;
                    case TURN_ARM:
                        intake.moveArm(ARM_LIMIT - 200);
                        intake.moveWrist(120);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.EXTEND;
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
                        delay = 0.7;
                        next_position = INTAKE_POSITIONS.TURN_CLAW;
                        previous_position = INTAKE_POSITIONS.LOWER_SLIDES_BEFORE_FOLD;
                        break;

                    case TURN_CLAW:
                        if (intake.slides.isBusy() && !move_next_override) {
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
                        if (intake.slides.getCurrentPosition() < 50 || move_next_override) {
                            intake.foldWrist();
                            intake.moveArm(200);
                            next_position = INTAKE_POSITIONS.START;
                            previous_position = INTAKE_POSITIONS.TURN_CLAW;
                            move_next = false;
                        }
                        break;
                }
            }


            //automatic distance pickup
            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR) || position.equals(INTAKE_POSITIONS.GRAB_FLOOR_SPECIMEN)){
//                distance.setOn(!disable_distances);

                if(!disable_distances && distance.getFilteredDist() > RobotConstants.TOO_CLOSE && distance.getFilteredDist() < RobotConstants.TOO_FAR){
                    if (waiting_for_distance){
                        if(timer.time() - distance_wait > 0.3) {
                            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR)) {
                                move_next2 = true;
                            }else{
                                move_next3 = true;
                            }
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
                move_next2_override = move_next2 && multiClick.getTaps("x") > 0;

                trafficLight.flashGreen(0.5, multiClick.getTaps("x"));

                if(multiClick.getTaps("x") > 1){
                    distance.setOn(!disable_distances);
                    mecaTank.setDistanceType(true);
                    if(mecaTank.getDistance() > 12 || disable_distances){
                        trafficLight.flashRed(0.5, 2);
                    }else {
                        drive_pid_on = true;

                        mecaTank.LivePIDToDistance(RobotConstants.TARGET);

                    }
                }
                else {
                    intake.enableLevel(false);
                    intake.setFourBar(false);
                    intake.setInsidePick(false);
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
                            }
                            break;
                        case EXIT_FLOOR_PICKUP_SEQ:
                            if(timer.time() - current_time > delay || move_next2_override){
                                move_next = true;
                                move_next2 = false;
                                next_position = INTAKE_POSITIONS.RAISE_ARM;
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

            //manual slide and arm control
            boolean left_bumper = currentGamepad1.left_bumper;
            boolean right_bumper = currentGamepad1.right_bumper;

            if (left_bumper) {
                intake.setSlidePower(-0.4);
            } else if (right_bumper) {
                intake.setSlidePower(0.4);
            }else if(currentGamepad2.right_stick_y != 0 && !reset_claw){
                intake.setSlidePower(-currentGamepad2.right_stick_y);
            }
            else {
                intake.setSlidePower(0);
            }
            boolean dpad_down = currentGamepad1.dpad_down;
            boolean dpad_up = currentGamepad1.dpad_up;
            if(dpad_down && !previousGamepad1.dpad_down && intake.getLevelOffset() <= 0 && maintain_level){
                intake.absIncrementLevelOffset(-30);
            }
            if (dpad_down) {
                if(maintain_level){
                    intake.incrementLevelOffset(-10);
                }else {
                    intake.setRotationPower(-0.4);
                }
            } else if (dpad_up) {
                if(maintain_level){
                    intake.incrementLevelOffset(10);
                }else{
                    intake.setRotationPower(0.4);
                }

            }
            else if(currentGamepad2.left_stick_y != 0){
                intake.setRotationPower(-currentGamepad2.left_stick_y);
            }
            else {
                intake.setRotationPower(0);
            }

            //specimen
            boolean y = (player2 && currentGamepad2.y && !previousGamepad2.y) || (currentGamepad1.y && !previousGamepad1.y);
            multiClick.update("y", getRuntime(), y);

            if(!mecaTank.isBusy() && drive_pid_on){
                drive_pid_on = false;
                distance.setOn(false);
                rear_distance.setOn(false);
            }
            if (multiClick.getTaps("y") > 0 || move_next3) {
                move_next3_override = move_next3 && multiClick.getTaps("y") > 0;
                move_next = false;
                move_next2 = false;
                trafficLight.flashGreen(0.5, multiClick.getTaps("y"));
                if(multiClick.getTaps("y") == 2 && !disable_distances){
                    mecaTank.setDistanceType(false);
                    rear_distance.setOn(!disable_distances);
                    mecaTank.LivePIDToDistance(4.5);
                    drive_pid_on = true;

                }
                else {
                    mecaTank.setMaxPower(1);
                    intake.slides.setMax(1400);
                    intake.enableLevel(false);

                    switch (position) {
                        case PICK_UP_FLOOR:
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

                            if (!mecaTank.isBusy() || move_next3_override) {
                                intake.setDistanceScoping(false);
                                distance.setOn(false);

                                intake.closeClaw();
                                delay = 0.4;
                                current_time = timer.time();
                                next_position3 = INTAKE_POSITIONS.RAISE_ARM_FOR_SPECIMEN;
                                move_next3 = true;
                            }
                            break;
                        case RAISE_ARM_FOR_SPECIMEN:
                            if (timer.time() - current_time > delay || move_next3_override) {
                                move_next3 = false;
                                intake.moveArm(ARM_LIMIT);
                                intake.moveSlides(200);
                                intake.moveWrist(RobotConstants.specimen_deliver);
                                next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
                            }
                            break;
                        case FORWARD:
                            if ((intake.arm.isBusy() || intake.slides.isBusy()) && !move_next3_override) break;

                            next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
                            break;
                        case SCORE_SPECMEN:
                            if (mecaTank.isBusy() && !move_next3_override) {
                                break;
                            }
                            intake.moveSlides(0);

                            intake.moveArm(ARM_LIMIT + 50);
                            if(disable_distances){
                                next_position3 = INTAKE_POSITIONS.RELEASE;
                            }else {
                                next_position3 = INTAKE_POSITIONS.REVERSE;
                            }
                            move_next3 = true;
                        case REVERSE:
                            if (intake.slides.isBusy() && !move_next3_override) {
                                break;
                            }
                            mecaTank.setDistanceType(false);
                            rear_distance.setOn(!disable_distances);
                            if(!disable_distances) {
                                mecaTank.DrivePastDistance(10, 0.4);
                            }
                            drive_pid_on = true;
                            next_position3 = INTAKE_POSITIONS.RELEASE;
                            break;
                        case RELEASE:
                            if (mecaTank.isBusy() && !move_next3_override) {
                                break;
                            }
                            rear_distance.setOn(false);

                            mecaTank.setDistanceType(true);
                            intake.openClaw();
                            move_next3 = false;
                            next_position3 = INTAKE_POSITIONS.FOLD_IN_AFTER_SPECIMEN;
                            break;
                        case FOLD_IN_AFTER_SPECIMEN:

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
                            next_position3 = INTAKE_POSITIONS.PICK_UP_FLOOR;
                            break;
                    }
                    position = next_position3;
                }
                multiClick.clearTaps("y");
            }

            if(telemetry_on) {
                mecaTank.telemetry();
                intake.telemetry();
                rear_distance.telemetry();
                distance.telemetry();
//                camera.telemetry();
            }
            multiClick.enableTelemetry(telemetry_on);


            intake.update();
            mecaTank.update();
            distance.update();
            rear_distance.update();
            if(loopTimer.milliseconds() > 100 && !disable_distances){
                disable_distances = true;
            }
            if(disable_distances){
                rear_distance.setOn(false);
                distance.setOn(false);
            }
            telemetry.addData("Front Distance On", distance.isOn());
            telemetry.addData("Intake Position", position);
            telemetry.addData("Rear Distance On", rear_distance.isOn());
            telemetry.addData("Rear Distance Count", rear_distance.getCalls());
            telemetry.addData("Distance Count", distance.getCalls());
            telemetry.addData("Distance Disabled", disable_distances);
            telemetry.addData("Cycle Time", loopTimer.milliseconds());
            telemetry.update();

        }
    }

}

