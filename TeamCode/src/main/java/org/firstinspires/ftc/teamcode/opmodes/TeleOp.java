package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        ALLOW_SLIDE_CONTROL,
        TURN_CLAW, PICK_UP_FLOOR, GRAB_FLOOR, FULL_RESET, RAISE_ARM
    };
    public static boolean player2 = false;
    boolean move_next = false;
    public static double target_height = -1.8;
    public static int offset = 0;
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;
    ElapsedTime timer;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    double fold_in_delay = 0;

    public void runOpMode(){
        mecaTank = new MecaTank(hardwareMap, telemetry);
        timer = new ElapsedTime();
        intake = new Intake(hardwareMap, telemetry, timer);

        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        intake.setTargetAngle(10);

        intake.init();
        mecaTank.init();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);

            intake.setTargetHeight(target_height);
            boolean b = (currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b);
            if(b){
                intake.moveSlides(intake.slides.getCurrentPosition());
                intake.moveArm(intake.arm.getCurrentPosition());
                next_position = INTAKE_POSITIONS.START;
                position = INTAKE_POSITIONS.ALLOW_SLIDE_CONTROL;

            }
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            boolean a = player2 ? (currentGamepad2.a && !previousGamepad2.a) : (currentGamepad1.a && !previousGamepad1.a);
            if(move_next || a){
                position = next_position;
                move_next = false;
                switch(next_position){
                    case START:
                        next_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;
                    case LOWER_CLAW:
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        break;
                    case CLOSE_AND_FOLD:
                        fold_in_delay = timer.seconds();
                        next_position = INTAKE_POSITIONS.RAISE_ARM;
                        break;
                    case RAISE_ARM:
                        fold_in_delay = timer.seconds();
                        next_position = INTAKE_POSITIONS.FOLD;
                        break;
                    case FOLD:
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE;
                        break;
                    case RETRACT_TO_LEAVE:
                        next_position = INTAKE_POSITIONS.TURN_ARM;
                        break;
                    case TURN_ARM:
                        next_position = INTAKE_POSITIONS.EXTEND;
                        break;
                    case EXTEND:
                        next_position = INTAKE_POSITIONS.TURN_CLAW;
                        break;
                    case TURN_CLAW:
                        fold_in_delay = timer.time();
                        next_position = INTAKE_POSITIONS.DELIVER;
                        break;
                    case DELIVER:
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE_2;
                        break;
                    case RETRACT_TO_LEAVE_2:
                        next_position = INTAKE_POSITIONS.FOLD_IN_2;
                        break;
                    case FOLD_IN_2:
                        next_position = INTAKE_POSITIONS.START;
                        break;



                }
            }
            boolean x = player2 ? (currentGamepad2.x && !previousGamepad2.x) : (currentGamepad1.x && !previousGamepad1.x);
            if(x){
                switch(position){
                    case PICK_UP_FLOOR:
                        position = INTAKE_POSITIONS.GRAB_FLOOR;
                        break;
                    case GRAB_FLOOR:
                        position = INTAKE_POSITIONS.FOLD;
                        next_position = INTAKE_POSITIONS.TURN_ARM;
                        break;
                    default:
                        position = INTAKE_POSITIONS.PICK_UP_FLOOR;
                        break;


                }
            }


            boolean left_bumper = player2 ? (currentGamepad2.left_bumper) : (currentGamepad1.left_bumper);
            boolean right_bumper = player2 ? (currentGamepad2.right_bumper ) : (currentGamepad1.right_bumper );
            if(left_bumper){
//                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
//                    intake.moveArmWithDelay(10);
//                }else {
                    intake.setSlidePower(-0.2);
//                }
            }else if(right_bumper){
//                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
//                    intake.moveArmWithDelay(-10);
//                }else {
                    intake.setSlidePower(0.2);
//                }
            }else{
                intake.setSlidePower(0);
            }
            boolean dpad_down = player2 ? currentGamepad2.dpad_down : currentGamepad1.dpad_down;
            boolean dpad_up = player2 ? currentGamepad2.dpad_up : currentGamepad1.dpad_up;
            if(dpad_down){
                if (position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
                    offset -= 10;
                }else {
                    intake.setRotationPower(-0.3);
                }
            }
            else if(dpad_up){
                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)) {
                    offset += 10;
                }else {
                    intake.setRotationPower(0.3);
                }
            }else{
                intake.setRotationPower(0);
            }

            boolean y = player2 ? currentGamepad2.y && !previousGamepad2.y : currentGamepad1.y && !previousGamepad1.y;
            if(y){
                next_position = INTAKE_POSITIONS.LOWER_CLAW;
                position = INTAKE_POSITIONS.START;
            }
            switch(position) {
                case PICK_UP_FLOOR:
                   intake.moveWrist(0.65);
                   intake.moveClaw(0.7);
                   break;
                case GRAB_FLOOR:
                    intake.closeClaw();
                    break;
                case START:
                    intake.plowClaw();
                    intake.foldWrist();
                    intake.moveArm(150);
                    intake.moveSlides(300);
                    break;
                case LOWER_CLAW:
                    intake.extendWrist();

                    intake.setFourBar(true);
//                    intake.moveArm(intake.calculateArmPosition(intake.slides.getCurrentPosition()) + offset);
                    break;

                case CLOSE_AND_FOLD:
                    intake.closeClaw();
                    if(timer.seconds() - fold_in_delay > 0.4){
                        move_next = true;
                    }
                    break;
                case RAISE_ARM:
                    intake.moveArm(300);
                    intake.setFourBar(false);
                    if(timer.seconds() - fold_in_delay > 0.4){
                        move_next = true;
                    }
                    break;
                case FOLD:
                    intake.moveWrist(0.8);
                    break;
                case RETRACT_TO_LEAVE:
                case RETRACT_TO_LEAVE_2:
                    intake.moveSlides(0);
                    position = INTAKE_POSITIONS.ALLOW_SLIDE_CONTROL;
                    break;
                case FOLD_IN_2:
                    if(!intake.slides.isBusy()) {
                        intake.foldWrist();
                        intake.moveArm(0);
                    }
                    break;
                case FOLD_IN:
                    intake.moveArm(300);
                    break;
                case TURN_ARM:
                    intake.moveArm(1460);
                    intake.extendWrist();
                    break;
                case EXTEND:
                    intake.moveSlides(1400);
                    position = INTAKE_POSITIONS.ALLOW_SLIDE_CONTROL;
                    move_next = true;
                    break;
                case TURN_CLAW:
                    if(!intake.slides.isBusy()) {
                        intake.foldWrist();
                    }
                    break;
                case DELIVER:

                    intake.openClaw();
                    intake.moveWrist(0.5);
                    if(timer.time() - fold_in_delay > 0.3) {
                        move_next = true;
                    }
                    break;
                case ALLOW_SLIDE_CONTROL:
                    break;



            }
            mecaTank.telemetry();
            intake.telemetry();

            intake.update();
            mecaTank.update();
            telemetry.update();
        }
    }
}
