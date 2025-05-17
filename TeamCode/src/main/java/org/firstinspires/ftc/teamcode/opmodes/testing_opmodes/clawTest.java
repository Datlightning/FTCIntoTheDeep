package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
@Config
@TeleOp
public class clawTest extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private DcMotor Arm;

    private Servo claw;
    private int targetPosition = 0;
    private final double ARM_POWER = 0.5;
    private final int MOVE_INCREMENT = 10;
    private final int MIN_POSITION = 0;
    private final int MAX_POSITION = 300;
    @Override
    public void runOpMode() throws InterruptedException{
        FrontLeft=hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight=hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft=hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight=hardwareMap.get(DcMotor.class, "BackRight");
        Arm=hardwareMap.get(DcMotor.class, "Arm");
        claw=hardwareMap.get(Servo.class, "claw");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();
        while(!isStopRequested()&& opModeIsActive()){
            FrontLeft.setPower(gamepad1.left_stick_y);
            BackLeft.setPower(gamepad1.left_stick_y);
            FrontRight.setPower(-gamepad1.right_stick_y);
            BackRight.setPower(-gamepad1.right_stick_y);


            if (gamepad1.dpad_up) {
                targetPosition += MOVE_INCREMENT;
            } else if (gamepad1.dpad_down) {
                targetPosition -= MOVE_INCREMENT;
            }
            targetPosition = clamp(targetPosition, MIN_POSITION, MAX_POSITION);
            Arm.setTargetPosition(targetPosition);
            Arm.setPower(ARM_POWER);
            telemetry.addData("Arm Current Position: ", Arm.getCurrentPosition());
            telemetry.addData("Arm Target Position: ", Arm.getTargetPosition());
            telemetry.update();

            sleep(20);

            if(gamepad1.a){
                openClaw();
            }else if(gamepad1.b){
                closeClaw();
            }
            }

        }
        private int clamp(int val, int min, int max) {
            return Math.max(min, Math.min(max, val));
        }
        public void openClaw(){
            PwmControl pwmControl = (PwmControl) claw;
            pwmControl.setPwmEnable();
            claw.setPosition(-0.4);
            sleep(1000);
            pwmControl.setPwmDisable();
        }
        public void closeClaw(){
            PwmControl pwmControl = (PwmControl) claw;
            pwmControl.setPwmEnable();
            claw.setPosition(0.4);
            sleep(1000);
            pwmControl.setPwmDisable();
        }

    }


