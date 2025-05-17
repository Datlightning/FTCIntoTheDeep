package org.firstinspires.ftc.teamcode.roadrunner.tuning;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
@TeleOp
public class ForwardPushTest extends LinearOpMode {
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    @Override
    public void runOpMode(){
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");

        int ticksAVG = ((FrontRight.getCurrentPosition()+FrontLeft.getCurrentPosition()+BackRight.getCurrentPosition()+BackLeft.getCurrentPosition())/4);

        telemetry.addData("FrontRight Ticks: ", FrontRight.getCurrentPosition());
        telemetry.addData("FrontLeft Ticks: ", FrontLeft.getCurrentPosition());
        telemetry.addData("FrontRight Ticks: ", BackRight.getCurrentPosition());
        telemetry.addData("FrontRight Ticks: ", BackLeft.getCurrentPosition());
        telemetry.addData("Average Ticks: ", ticksAVG);
    }
}
