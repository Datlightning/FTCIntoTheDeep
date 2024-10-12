package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;


@TeleOp
@Config
public class MecanumEarlyTest extends LinearOpMode {
    Intake intake;
    MecaTank mecaTank;
    private enum INTAKE_POSITIONS {
        START,
        LOWER_CLAW,
        REACH,
        RETRACT,
        FOLD,
        CLOSE_AND_FOLD,
        RETRACT_TO_LEAVE,
        RETRACT_TO_LEAVE_2,
        FOLD_IN,
        FOLD_IN_2,
        REACH_AND_EXTEND,
        DELIVER
    };
    boolean gamepad1_a_toggle = false;
    boolean gamepad1_x_toggle = false;
    boolean move_next = false;
    INTAKE_POSITIONS position = INTAKE_POSITIONS.FOLD_IN;
    INTAKE_POSITIONS next_position = INTAKE_POSITIONS.START;
    ElapsedTime timer;
    double fold_in_delay = 0;
    public void runOpMode(){
        mecaTank = new MecaTank(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        timer = new ElapsedTime();
        intake.init();
        mecaTank.init();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            mecaTank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);
            if(move_next || (gamepad1.a && !gamepad1_a_toggle)){
                position = next_position;
                move_next = false;
                switch(next_position){
                    case START:
                        next_position = INTAKE_POSITIONS.LOWER_CLAW;
                        break;
                    case LOWER_CLAW:
                        next_position = INTAKE_POSITIONS.REACH;
                        break;
                    case REACH:
                        fold_in_delay = timer.seconds();
                        next_position = INTAKE_POSITIONS.CLOSE_AND_FOLD;
                        break;
                    case CLOSE_AND_FOLD:
                        next_position = INTAKE_POSITIONS.FOLD;
                        break;
                    case FOLD:
                        next_position = INTAKE_POSITIONS.RETRACT_TO_LEAVE;
                        break;
                    case RETRACT_TO_LEAVE:
                        next_position = INTAKE_POSITIONS.FOLD_IN;
                        break;
                    case FOLD_IN:
                        next_position = INTAKE_POSITIONS.REACH_AND_EXTEND;
                        break;
                    case REACH_AND_EXTEND:
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
                gamepad1_a_toggle = true;
            }else if(!gamepad1.a){
                gamepad1_a_toggle = false;
            }
            if(!gamepad1_x_toggle && gamepad1.x){
                gamepad1_x_toggle = true;

                if(position.equals(INTAKE_POSITIONS.REACH)) {
                    position = INTAKE_POSITIONS.RETRACT;
                    next_position = INTAKE_POSITIONS.REACH;
                }
            }else if(!gamepad1.x){
                gamepad1_x_toggle = false;
            }
            switch(position) {
                case START:
                    intake.moveClaw(0.1);
                    intake.moveWrist(1);
                    intake.moveSlides(150);
                    intake.moveArm(200);
                    break;
                case LOWER_CLAW:
                    intake.moveWrist(0.06);
                    break;
                case REACH:
                    intake.moveSlides(400);
                    break;
                case RETRACT:
                    intake.moveSlides(150);
                    break;
                case CLOSE_AND_FOLD:
                    intake.moveClaw(0.3);
                    if(timer.seconds() - fold_in_delay > 0.3){
                        move_next = true;
                    }
                    break;
                case FOLD:
                    intake.moveWrist(0.5);
                    break;
                case RETRACT_TO_LEAVE:
                case RETRACT_TO_LEAVE_2:
                    intake.moveSlides(0);
                    break;
                case FOLD_IN:
                case FOLD_IN_2:
                    intake.moveWrist(1);
                    intake.moveArm(0);
                    intake.moveClaw(0.27);
                    break;

                case REACH_AND_EXTEND:
                    intake.moveArm(1410);
                    intake.moveSlides(900);
                    break;
                case DELIVER:
                    intake.moveClaw(0);
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
