package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecaTank;
import org.firstinspires.ftc.teamcode.subsystems.TrafficLight;

@TeleOp
@Config
public class IAmStressTestingTheSlidesSoShreyasCanFindOutWhatsWrong extends TestingOpMode {
    Intake intake;
    public static int bottom = 0;
    public static int top = 1200;
    public static int arm_pos = 1400;
    private boolean up = false;
    @Override
    public void runOpMode() throws InterruptedException {
        makeTelemetry();
        intake = new Intake(hardwareMap, telemetry);
        intake.init();
        intake.arm.setUseMotionProfile(false);
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            intake.moveArm(arm_pos);
            if(!intake.arm.isBusy()) {
                telemetry.addLine("Slides go brr");
                if (!intake.slides.isBusy()) {
                    intake.moveSlides(up ? bottom : top);
                    up = !up;
                }
            }
            intake.update();
            intake.telemetry();
            telemetry.update();
        }

    }
}
