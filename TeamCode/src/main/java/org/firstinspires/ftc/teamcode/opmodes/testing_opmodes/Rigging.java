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
public class Rigging extends LinearOpMode{
    private DcMotor LeftFrontMotor;
    private DcMotor LeftBackMotor;
    private DcMotor RightFrontMotor;
    private DcMotor RightBackMotor;
    private DcMotor RiggingArm;
    @Override
    public void runOpMode() throws InterruptedException{
        LeftFrontMotor=hardwareMap.get(DcMotor.class, "LFM");
        LeftBackMotor=hardwareMap.get(DcMotor.class, "LBM");
        RightFrontMotor=hardwareMap.get(DcMotor.class, "RFM");
        RightBackMotor=hardwareMap.get(DcMotor.class, "RBM");
        RiggingArm=hardwareMap.get(DcMotor.class, "RiggingArm");
        waitForStart();
        while(!isStopRequested()&& opModeIsActive()) {

        }
}
}
