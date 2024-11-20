package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;

    TrafficLight trafficLight;

    private enum INTAKE_POSITIONS {
        START,
        LOWER_CLAW,
        TURN_ARM,
        EXTEND,
        FOLD,
        CLOSE_AND_FOLD,
        RETRACT_TO_LEAVE,
        RETRACT_TO_LEAVE_2,
        FOLD_IN,
        FOLD_IN_2,
        DELIVER,
        TURN_CLAW,
        PICK_UP_FLOOR,
        GRAB_FLOOR,
        RAISE_CLAW,
         DROP_ARM, SCORE_SPECMEN, RELEASE, FOLD_IN_AFTER_SPECIMEN, RAISE_ARM_FOR_SPECIMEN, GRAB_FLOOR_SPECIMEN, REVERSE, FORWARD, RAISE_ARM
    }


    public static boolean player2 = false;
    public static double target_height = 3;
    boolean move_next = false;
    boolean move_next2 = false;
    boolean move_next3 = false;
    public static int offset = 0;


    public static boolean telemetry_on = false;

    double lastTapTimeA = 0;

    int tapsA = 0;
    int current_tapsA = 0;

    double lastTapTimeX = 0;

    int tapsX = 0;
    int current_tapsX = 0;

    double lastTapTimeY = 0;

    int tapsY = 0;
    int current_tapsY = 0;

    public static double current_closed = 0.95;
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;

    INTAKE_POSITIONS previous_position = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS next_position2 = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS next_position3 = INTAKE_POSITIONS.START;
    ElapsedTime timer;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    double current_time = 0;
    double delay = 0;

    private boolean waiting_for_distance = false;
    double distance_wait = 0;


    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        trafficLight = new TrafficLight("Traffic Light", hardwareMap, telemetry, RobotConstants.red_led, RobotConstants.green_led);

        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer, trafficLight);
        mecaTank = new MecaTank(hardwareMap, telemetry, timer, trafficLight);

        if(RobotConstants.auto_transfer){
            offset = RobotConstants.offset;
            intake.init_without_encoder_reset();
//            mecaTank = new MecaTank(hardwareMap, telemetry, timer, RobotConstants.imu);


        }else {

            intake.init();
            sleep(200);
            intake.calculateOffset();
            offset = intake.getOffset();
        }

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        intake.setTargetAngle(90);
        intake.arm.setUseMotionProfile(true);
        intake.slides.setUseMotionProfile(true);

        mecaTank.init();

        waitForStart();
        intake.moveArm(250);
        while (!isStopRequested() && opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);

            intake.setTargetHeight(target_height);
            boolean b = (currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b);
            if (b) {
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
            boolean dpad_right = player2 ?  (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) : (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
            boolean dpad_left = player2 ?  (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) : (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);

            if(dpad_left){
                intake.plowClaw();
            }
            if(dpad_right){
                intake.closeClaw();
            }
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

            if(getRuntime() - lastTapTimeA > 0.3) {
                current_tapsA = tapsA;
                lastTapTimeA = getRuntime();
                tapsA = 0;

            }
            boolean a = player2 ? (currentGamepad2.a && !previousGamepad2.a) : (currentGamepad1.a && !previousGamepad1.a);

            if(a){
                tapsA += 1;
                lastTapTimeA = getRuntime();
            }

            if (move_next || current_tapsA > 0) {
                trafficLight.flashGreen(0.5, current_tapsA);

                // Move forward if single tap, move backward if double tap
                if (current_tapsA == 1 || move_next) {
                    telemetry.addLine("progress forward");
                    position = next_position;  // Move forward
                } else if (current_tapsA == 2) {
                    telemetry.addLine("run.");
                    position = previous_position;  // Move backward
                }
                current_tapsA = 0;

                switch (position) {
                    case START:
                        move_next = false;
                        intake.plowClaw();
                        intake.moveWrist(0.62);
                        intake.moveArm(250);
                        intake.moveSlides(300);
                        intake.slides.setMax(1000);
                        intake.setFourBar(false);
                        intake.enableLevel(false);

                        next_position = INTAKE_POSITIONS.LOWER_CLAW;
                        previous_position = INTAKE_POSITIONS.START; // Set previous position for double tap
                        break;

                    case LOWER_CLAW:
                        intake.extendWrist();
                        intake.plowClaw();
                        intake.moveArm(250);
                        intake.setTargetAngle(90);
                        intake.setFourBar(true);
                        intake.arm.setUseMotionProfile(false);
                        intake.enableLevel(true);
                        next_position = INTAKE_POSITIONS.DROP_ARM;
                        previous_position = INTAKE_POSITIONS.START;
                        break;

                    case DROP_ARM:
                        intake.useFastPID(true);
                        intake.enableLevel(true);
                        intake.setFourBar(true);
                        intake.arm.setManualPower(-0.2);
                        delay = 0.2;
                        current_time = timer.time();
                        move_next = true;
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;

                    case CLOSE_AND_FOLD:
                        if (timer.time() - current_time < delay) {
                            break;
                        }
                        move_next = false;
                        intake.setFourBar(false);

                        mecaTank.setMaxPower(1);
                        current_time = timer.seconds();
                        intake.closeClaw();
                        next_position = INTAKE_POSITIONS.RAISE_ARM;
                        intake.slides.setMax(1400);
                        intake.enableLevel(false);

                        previous_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;

                    case RAISE_ARM:

                        move_next = true;
                        intake.slides.setMax(1400);
                        current_time = timer.seconds();
                        intake.moveWrist(0.5);
                        intake.useFastPID(true);
                        intake.moveArm(250);
                        intake.setFourBar(false);
                        next_position = INTAKE_POSITIONS.FOLD;
                        previous_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        break;

                    case FOLD:
                        if (intake.arm.getCurrentPosition() <= 200) {
                            break;
                        }
                        move_next = false;
                        intake.arm.setUseMotionProfile(true);
                        intake.useFastPID(false);
                        intake.moveWrist(0.6);
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.TURN_ARM;
                        previous_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        break;

                    case TURN_ARM:
                        intake.moveArm(1450 + intake.getOffset());
                        intake.moveWrist(0.5);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.EXTEND;
                        previous_position = INTAKE_POSITIONS.FOLD;
                        break;

                    case EXTEND:
                        if (intake.arm.getCurrentPosition() < 1380 + intake.getOffset() - 200) {
                            break;
                        }
                        intake.slides.setMax(1400);
                        intake.moveSlides(1200);
                        mecaTank.setMaxPower(0.5);
                        current_time = timer.time();
                        delay = 0.7;
                        next_position = INTAKE_POSITIONS.TURN_CLAW;
                        previous_position = INTAKE_POSITIONS.FOLD;
                        break;

                    case TURN_CLAW:
                        if (intake.slides.isBusy()) {
                            break;
                        }
                        move_next = false;
                        intake.moveWrist(0.8);
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
                        if (timer.time() - current_time > delay) {
                            break;
                        }
                        current_time = timer.time();
                        delay = 0.1;
                        intake.moveWrist(0.5);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE_2;
                        previous_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;

                    case RETRACT_TO_LEAVE_2:
                        if (timer.time() - current_time > delay) {
                            break;
                        }
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.FOLD_IN_2;
                        previous_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;

                    case FOLD_IN_2:
                        if (intake.slides.getCurrentPosition() < 50) {
                            intake.foldWrist();
                            intake.moveArm(180 + offset);
                            next_position = INTAKE_POSITIONS.START;
                            previous_position = INTAKE_POSITIONS.TURN_CLAW;
                            move_next = false;
                        }
                        break;
                }
            }


            boolean x = player2 ? (currentGamepad2.x && !previousGamepad2.x) : (currentGamepad1.x && !previousGamepad1.x);

            if(x){
                tapsX += 1;
                lastTapTimeX = getRuntime();
            }
            if(getRuntime() - lastTapTimeX > 0.3) {
                current_tapsX = tapsX;
                lastTapTimeX = getRuntime();
                tapsX = 0;

            }
            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR) || position.equals(INTAKE_POSITIONS.GRAB_FLOOR_SPECIMEN)){
                if(intake.distance.getFilteredDist() > RobotConstants.TOO_CLOSE && intake.distance.getFilteredDist() < RobotConstants.TOO_FAR){
                    if (waiting_for_distance){
                        if(timer.time() - distance_wait > 0.3) {
                            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR)) {
                                move_next2 = true;
                            }else{
                                move_next3 = true;
                            }
                        }

                    }else {
                        waiting_for_distance = true;
                        distance_wait = timer.time();
                    }
//                    telemetry.addData("Distance Waiting Time", timer.time() - distance_wait);
//                    telemetry.addData("Distance Wait", distance_wait);
                }else{
                    waiting_for_distance = false;
                }
            }
//            telemetry.addData("Waiting For Distance", waiting_for_distance);
//
//            telemetry.addData("Intake Position", position);



            if (current_tapsX > 0 || move_next2) {
                trafficLight.flashGreen(0.5, current_tapsX);

                if(current_tapsX == 2){
                    mecaTank.setDistanceType(true);
                    if(mecaTank.getDistance() > 13){
                        trafficLight.flashRed(0.5, 2);
                    }else {
                        mecaTank.PIDToDistance(RobotConstants.TARGET);
                    }
                } else if(current_tapsX == 1) {
                    intake.enableLevel(false);
                    intake.setFourBar(false);

                    mecaTank.setMaxPower(1);
                    intake.slides.setMax(1400);

                    switch (position) {
                        case PICK_UP_FLOOR:
                            if (!intake.slides.isBusy()) {
                                intake.setDistanceScoping(true);
                                intake.moveArm(0);
                                next_position2 = INTAKE_POSITIONS.GRAB_FLOOR;
                                move_next2 = false;
                            }
                            break;
//
                        case GRAB_FLOOR:
                            if (!mecaTank.isBusy()) {
                                intake.setDistanceScoping(false);

                                move_next2 = false;
                                intake.closeClaw();
                                move_next = true;
                                delay = 0.3;
                                current_time = timer.time();
                                next_position = INTAKE_POSITIONS.RAISE_ARM;
                            }
                            break;
                        default:
                            intake.moveSlides(0);
                            intake.moveWrist(RobotConstants.floor_pickup_position);
                            intake.moveClaw(0.8);

                            next_position2 = INTAKE_POSITIONS.PICK_UP_FLOOR;
                            move_next2 = true;
                            break;
                    }
                    position = next_position2;
                }
                current_tapsX = 0;


                }


            boolean left_bumper = player2 ? (currentGamepad2.left_bumper) : (currentGamepad1.left_bumper);
            boolean right_bumper = player2 ? (currentGamepad2.right_bumper) : (currentGamepad1.right_bumper);
            if (left_bumper) {
                intake.setSlidePower(-0.4);
            } else if (right_bumper) {
                intake.setSlidePower(0.4);
            } else {
                intake.setSlidePower(0);
            }
            boolean dpad_down = player2 ? currentGamepad2.dpad_down : currentGamepad1.dpad_down;
            boolean dpad_up = player2 ? currentGamepad2.dpad_up : currentGamepad1.dpad_up;
            if (dpad_down) {
//                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
//                    offset -= 20;
//                    intake.moveArm(intake.arm.targetPos - 20);
//                }else {
                    intake.setRotationPower(-0.4);
                    telemetry.addData("Move up", -10);
                    telemetry.update();
//                }
            } else if (dpad_up) {
//                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
//                    offset += 20;
//                    intake.moveArm(intake.arm.targetPos + 20);
//                }else {
                    intake.setRotationPower(0.4);
                telemetry.addData("Move up", 10);
                telemetry.update();
//                }
            } else {
                intake.setRotationPower(0);
            }


            boolean y = player2 ? currentGamepad2.y && !previousGamepad2.y : currentGamepad1.y && !previousGamepad1.y;
            if(y){
                tapsY += 1;
                lastTapTimeY = getRuntime();
            }
            if(getRuntime() - lastTapTimeY > 0.3) {
                current_tapsY = tapsY;
                lastTapTimeY = getRuntime();
                tapsY = 0;

            }
            if (current_tapsY > 0 || move_next3) {
                trafficLight.flashGreen(0.5, current_tapsX);
                if(current_tapsY == 2){
                    mecaTank.setDistanceType(false);
                    mecaTank.PIDToDistance(4);
                }else {
                    mecaTank.setMaxPower(1);
                    intake.slides.setMax(1400);
                    intake.enableLevel(false);

                    switch (position) {
                        case PICK_UP_FLOOR:
                            if (!intake.slides.isBusy()) {
                                intake.setDistanceScoping(true);
                                intake.moveArm(0);
                                next_position3 = INTAKE_POSITIONS.GRAB_FLOOR_SPECIMEN;
                                move_next3 = false;
                            }
                            break;
                        case GRAB_FLOOR_SPECIMEN:
                            if (!mecaTank.isBusy()) {
                                intake.setDistanceScoping(false);
                                intake.closeClaw();
                                delay = 0.4;
                                current_time = timer.time();
                                next_position3 = INTAKE_POSITIONS.RAISE_ARM_FOR_SPECIMEN;
                                move_next3 = true;
                            }
                            break;
                        case RAISE_ARM_FOR_SPECIMEN:
                            if (timer.time() - current_time > delay) {
                                move_next3 = false;
                                intake.moveArm(1400);
                                intake.moveSlides(400);
                                intake.moveWrist(0.8);
                                next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
                            }
                            break;
                        case FORWARD:
                            if (intake.arm.isBusy() || intake.slides.isBusy()) break;

                            next_position3 = INTAKE_POSITIONS.SCORE_SPECMEN;
                            break;
                        case SCORE_SPECMEN:
                            if (mecaTank.isBusy()) {
                                break;
                            }
                            intake.moveSlides(100);
                            next_position3 = INTAKE_POSITIONS.REVERSE;
                            move_next3 = true;
                        case REVERSE:
                            if (intake.slides.isBusy()) {
                                break;
                            }
                            mecaTank.setDistanceType(false);
                            mecaTank.DrivePastDistance(9, 0.15);
                            next_position3 = INTAKE_POSITIONS.RELEASE;
                            break;
                        case RELEASE:
                            if (mecaTank.isBusy()) {
                                break;
                            }
                            mecaTank.setDistanceType(true);
                            intake.openClaw();
                            move_next3 = false;
                            next_position3 = INTAKE_POSITIONS.FOLD_IN_AFTER_SPECIMEN;
                            break;
                        case FOLD_IN_AFTER_SPECIMEN:

                            intake.moveArm(0);
                            intake.moveWrist(0.9);
                            next_position = INTAKE_POSITIONS.START;
                            next_position3 = INTAKE_POSITIONS.START;
                            break;
                        default:
                            intake.moveSlides(0);
                            intake.moveWrist(RobotConstants.floor_pickup_position);
                            intake.moveClaw(0.8);
                            move_next3 = true;
                            next_position3 = INTAKE_POSITIONS.PICK_UP_FLOOR;
                            break;
                    }
                    position = next_position3;
                }
                current_tapsY = 0;
            }

            if(telemetry_on) {
                mecaTank.telemetry();
                intake.telemetry();
            }

            intake.update();
            mecaTank.update();
            telemetry.update();
        }
    }

}

