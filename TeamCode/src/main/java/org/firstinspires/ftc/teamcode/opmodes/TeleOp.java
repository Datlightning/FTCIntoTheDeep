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
        RAISE_ARM
    };

    boolean move_next = false;
    public static double target_height = .125;
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;
    ElapsedTime timer;
    Gamepad currentGamepad1, currentGamepad2, previousGamepad1, previousGamepad2;
    double fold_in_delay = 0;
    public void runOpMode(){
        mecaTank = new MecaTank(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        timer = new ElapsedTime();
        currentGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad1 = new Gamepad();
        previousGamepad2 = new Gamepad();

        intake.setTargetAngle(90);

        intake.init();
        mecaTank.init();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad2.copy(gamepad2);
            currentGamepad1.copy(gamepad1);

            intake.setTargetHeight(target_height);
            if(currentGamepad1.b && !previousGamepad1.b){
                intake.moveSlides(intake.slides.getCurrentPosition());
                intake.moveArm(intake.arm.getCurrentPosition());
            }
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            if(move_next || (currentGamepad1.a && !previousGamepad1.a)){
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
                        next_position = INTAKE_POSITIONS.FOLD_IN;
                        break;
                    case FOLD_IN:
                        next_position = INTAKE_POSITIONS.TURN_ARM;
                        break;
                    case TURN_ARM:
                        next_position = INTAKE_POSITIONS.EXTEND;
                        break;
                    case EXTEND:
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

            if(currentGamepad1.left_bumper){
                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
                    intake.moveArmWithDelay(10);
                }else {
                    intake.setSlidePower(0.3);
                }
            }else if(currentGamepad1.right_bumper){
                if(position.equals(INTAKE_POSITIONS.LOWER_CLAW)){
                    intake.moveArmWithDelay(-10);
                }else {
                    intake.setSlidePower(-0.3);
                }
            }else{
                intake.setSlidePower(0);
            }
            if(currentGamepad1.dpad_down){
                intake.setRotationPower(-0.3);
            }
            else if(currentGamepad1.dpad_up){
                intake.setRotationPower(0.3);
            }else{
                intake.setRotationPower(0);
            }
            switch(position) {
                case START:
                    intake.plowClaw();
                    intake.foldWrist();
                    intake.moveArm(300);
                    intake.moveSlides(300);
                    break;
                case LOWER_CLAW:
                    intake.extendWrist();

                    intake.setFourBar(true);
                    break;

                case CLOSE_AND_FOLD:
                    intake.closeClaw();
                    if(timer.seconds() - fold_in_delay > 0.4){
                        move_next = true;
                    }
                    break;
                case RAISE_ARM:
                    intake.moveArm(600);
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
                case FOLD_IN:
                case FOLD_IN_2:
                    intake.foldWrist();
                    intake.moveArm(0);
                    break;
                case TURN_ARM:
                    intake.moveArm(1460);
                    break;
                case EXTEND:
                    if(intake.arm.getCurrentPosition() > 700){
                        intake.moveSlides(900);
                        position = INTAKE_POSITIONS.ALLOW_SLIDE_CONTROL;

                    }
                    break;
                case DELIVER:
                    intake.openClaw();
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
