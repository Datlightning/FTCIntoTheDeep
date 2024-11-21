package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class BigBoyTesting extends TestingOpMode {
    protected DcMotor motor1,motor2,motor3,motor4, motor1e, motor2e, motor3e, motor4e;
    public static double motor1power = 0,motor2power = 0, motor3power = 0,motor4power = 0,motor1epower = 0,motor2epower = 0,motor3epower = 0,motor4epower = 0;
    protected Servo servo1, servo2, servo3,servo4,servo5,servo6, servo1e,servo2e,servo3e,servo4e,servo5e, servo6e;
    public static double servo1_pwm = 0,
    servo2_pwm = 0,
    servo3_pwm = 0,
    servo4_pwm = 0,
    servo5_pwm = 0,
    servo6_pwm = 0,
    servo1e_pwm = 0,
    servo2e_pwm = 0,
    servo3e_pwm = 0,
    servo4e_pwm = 0,
    servo5e_pwm = 0,
    servo6e_pwm = 0;
    public static boolean gamepad_1_motor1 = false,
    gamepad_1_motor2 = false,
    gamepad_1_motor3 = false,
    gamepad_1_motor4 = false,
    gamepad_1_motor1e = false,
    gamepad_1_motor2e = false,
    gamepad_1_motor3e = false,
    gamepad_1_motor4e = false;

    public static double SCALAR = 3.0;




    public void runOpMode(){
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        motor1e = hardwareMap.get(DcMotor.class, "motor1e");
        motor2e = hardwareMap.get(DcMotor.class, "motor2e");
        motor3e = hardwareMap.get(DcMotor.class, "motor3e");
        motor4e = hardwareMap.get(DcMotor.class, "motor4e");

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");
        servo6 = hardwareMap.get(Servo.class, "servo6");
        servo1e = hardwareMap.get(Servo.class, "servo1e");
        servo2e = hardwareMap.get(Servo.class, "servo2e");
        servo3e = hardwareMap.get(Servo.class, "servo3e");
        servo4e = hardwareMap.get(Servo.class, "servo4e");
        servo5e = hardwareMap.get(Servo.class, "servo5e");
        servo6e = hardwareMap.get(Servo.class, "servo6e");

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){

            servo1.setPosition(servo1_pwm);
            servo2.setPosition(servo2_pwm);
            servo3.setPosition(servo3_pwm);
            servo4.setPosition(servo4_pwm);
            servo5.setPosition(servo5_pwm);
            servo6.setPosition(servo6_pwm);

            servo1e.setPosition(servo1e_pwm);
            servo2e.setPosition(servo2e_pwm);
            servo3e.setPosition(servo3e_pwm);
            servo4e.setPosition(servo4e_pwm);
            servo5e.setPosition(servo5e_pwm);
            servo6e.setPosition(servo6e_pwm);

            if(gamepad_1_motor1){
                motor1.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor1.setPower(motor1power);
            }
            if(gamepad_1_motor2){
                motor2.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor2.setPower(motor2power);
            }

            if(gamepad_1_motor3){
                motor3.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor3.setPower(motor3power);
            }

            if(gamepad_1_motor4){
                motor4.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor4.setPower(motor4power);
            }

            if(gamepad_1_motor1e){
                motor1e.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor1e.setPower(motor1epower);
            }
            if(gamepad_1_motor2e){
                motor2e.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor2e.setPower(motor2epower);
            }

            if(gamepad_1_motor3e){
                motor3e.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor3e.setPower(motor3epower);
            }

            if(gamepad_1_motor4e){
                motor4e.setPower(-gamepad1.left_stick_y/SCALAR);
            }else{
                motor4e.setPower(motor4epower);
            }



            telemetry.update();

        }


    }
}
