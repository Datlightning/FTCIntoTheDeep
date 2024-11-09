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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;

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
        TURN_CLAW, PICK_UP_FLOOR, GRAB_FLOOR, LOWER_CLAW_TO_CLEAR, RAISE_CLAW, AUTO_GRAB, RAISE_ARM_HIGHER, DISABLE_ARM, EXTEND_SLIDE, DROP_ARM, RAISE_ARM
    }

    ;
    public static boolean player2 = false;
    public static double target_height = 3;
    boolean move_next = false;
    boolean move_next2 = false;
    public static int offset = 0;

    public static double current_closed = 0.95;
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;
    INTAKE_POSITIONS next_position2 = INTAKE_POSITIONS.START;
    ElapsedTime timer;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    double current_time = 0;
    double delay = 0;

    private boolean waiting_for_distance = false;
    double distance_wait = 0;


    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer);
        mecaTank = new MecaTank(hardwareMap, telemetry, timer);

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
        intake.moveArm(500);
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
                intake.enableLevel(false);

            }
            boolean dpad_right = player2 ?  (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) : (currentGamepad1.dpad_right && !previousGamepad1.dpad_right);
            if(dpad_right){
                intake.moveWrist(0.55);//TODO: Clear level
            }
            boolean dpad_left = player2 ?  (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) : (currentGamepad1.dpad_left && !previousGamepad1.dpad_left);
            if(dpad_left){
                intake.moveWrist(0.43);//TODO: Grabby level
            }

            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            boolean a = player2 ? (currentGamepad2.a && !previousGamepad2.a) : (currentGamepad1.a && !previousGamepad1.a);
            if ( move_next || a) {

                position = next_position;

                switch (position) {
                    case START:
                        move_next = false;
                        intake.plowClaw();
                        intake.foldWrist();
                        intake.moveArm(250);
                        intake.moveSlides(300);
                        intake.slides.setMax(800);
                        next_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;
                    case LOWER_CLAW:
                        intake.extendWrist();
                        intake.moveArm(0);
                        intake.setFourBar(true);
                        intake.arm.setUseMotionProfile(false);
                        intake.enableLevel(true);
                        next_position = INTAKE_POSITIONS.DROP_ARM;
                        break;
                    case DROP_ARM:
                        intake.useFastPID(true);
                        intake.moveArm(intake.arm.getCurrentPosition() - 50);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        break;

                    case CLOSE_AND_FOLD:
                        if(intake.arm.isBusy()){
                            break;
                        }
                        move_next = false;
                        intake.setFourBar(false);

                        mecaTank.setMaxPower(1);
                        current_time = timer.seconds();
                        delay = 0.4;
                        intake.closeClaw();
                        next_position = INTAKE_POSITIONS.RAISE_ARM;
                        intake.slides.setMax(1400);
                        intake.enableLevel(false);


                        move_next = true;
                        break;
                    case RAISE_ARM:
                        if(timer.time() - current_time > delay){
                            break;
                        }
                        move_next = true;
                        intake.slides.setMax(1400);
                        current_time = timer.seconds();
                        intake.moveWrist(0.5);
                        intake.useFastPID(true);
                        intake.moveArm(250);
                        intake.setFourBar(false);
                        next_position = INTAKE_POSITIONS.FOLD;
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
                        break;
                    case TURN_ARM:
                        intake.moveArm(1380 + intake.getOffset());
                        intake.moveWrist(0.5);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.EXTEND;
                        break;
                    case EXTEND:
                        if(intake.arm.getCurrentPosition() < 1380 + intake.getOffset() - 200){
                            break;
                        }
                        intake.slides.setMax(1400);
                        intake.moveSlides(1400);
                        mecaTank.setMaxPower(0.5);
                        current_time = timer.time();
                        delay = 0.7;
                        next_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;
                    case TURN_CLAW:
                        if(intake.slides.isBusy()){
                            break;
                        }
                        move_next = false;
                        intake.moveWrist(0.8);
                        next_position = INTAKE_POSITIONS.DELIVER;
                        break;
                    case DELIVER:
                        mecaTank.setMaxPower(1);
                        intake.openClaw();
                        delay = 0.3;
                        current_time = timer.time();
                        move_next = true;
                        next_position = INTAKE_POSITIONS.RAISE_CLAW;
                        break;
                    case RAISE_CLAW:
                        if(timer.time() - current_time > delay){
                            break;
                        }
                        current_time = timer.time();
                        delay = 0.1;
                        intake.moveWrist(0.5);
                        move_next = true;
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE_2;
                        break;

                    case RETRACT_TO_LEAVE_2:
                        if(timer.time() - current_time > delay){
                            break;
                        }
                        intake.moveSlides(0);
                        next_position = INTAKE_POSITIONS.FOLD_IN_2;
                        break;
                    case FOLD_IN_2:
                        if (intake.slides.getCurrentPosition() < 50) {
                            intake.foldWrist();
                            intake.moveArm(180 + offset);
                            next_position = INTAKE_POSITIONS.START;
                            move_next = false;
                        }
                        break;


                }
            }

            boolean x = player2 ? (currentGamepad2.x && !previousGamepad2.x) : (currentGamepad1.x && !previousGamepad1.x);
            if(position.equals(INTAKE_POSITIONS.GRAB_FLOOR)){
                if(intake.distance.getFilteredDist() > RobotConstants.TOO_CLOSE && intake.distance.getFilteredDist() < RobotConstants.TOO_FAR){
                    if (waiting_for_distance){
                        if(timer.time() - distance_wait > 0.3) {
                            move_next2 = true;
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

            if (x || move_next2) {
                mecaTank.setMaxPower(1);
                intake.slides.setMax(1400);

                switch (position) {
                    case PICK_UP_FLOOR:
                        if(!intake.slides.isBusy()){
                            intake.moveArm(0);
                            next_position2 = INTAKE_POSITIONS.GRAB_FLOOR;
                            move_next2 = false;
                        }
                        break;
//                    case AUTO_GRAB:
//                        mecaTank.PIDToDistance(4);
//                        move_next2 = true;
//                        next_position2 = INTAKE_POSITIONS.GRAB_FLOOR;
//                        break;
                    case GRAB_FLOOR:
                        if(!mecaTank.isBusy()) {
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
                        intake.moveClaw(0.65);

                        next_position2 = INTAKE_POSITIONS.PICK_UP_FLOOR;
                        move_next2 = true;
                        break;
                }
                position = next_position2;

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
            if (y) {
                next_position = INTAKE_POSITIONS.START;
                delay = 0;
                move_next = true;
            }

            mecaTank.telemetry();
            intake.telemetry();

            intake.update();
            mecaTank.update();
            telemetry.update();
        }
    }
}

