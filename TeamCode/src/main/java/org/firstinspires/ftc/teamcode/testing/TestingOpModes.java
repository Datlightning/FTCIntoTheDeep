/*
package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.LocalizationTest;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.IntakeTuning;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.ManualLateralFeedbackTuner;
import org.firstinspires.ftc.teamcode.roadrunner.tuning.SplineTest;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class TestingOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final String GROUP = "testing";
    public static final boolean DISABLED = false;

    private TestingOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        manager.register(metaForClass(BigBoyTesting.class), BigBoyTesting.class);
        manager.register(metaForClass(DistanceRRPathTest.class), DistanceRRPathTest.class);
        manager.register(metaForClass(DriveTesting.class), DriveTesting.class);
        manager.register(metaForClass(EpicWebTransferTest.class), EpicWebTransferTest.class);
        manager.register(metaForClass(IndividualWheelTesting.class), IndividualWheelTesting.class);
        manager.register(metaForClass(IntakeTesting.class), IntakeTesting.class);
        manager.register(metaForClass(IntakeTuning.class), IntakeTuning.class);
        manager.register(metaForClass(PIDTuning.class), PIDTuning.class);
        manager.register(metaForClass(RRPathTest.class), RRPathTest.class);
        manager.register(metaForClass(ServoExTesting.class), ServoExTesting.class);


        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    BigBoyTesting.class,
                    DistanceRRPathTest.class,
                    DriveTesting.class,
                    EpicWebTransferTest.class,
                    IndividualWheelTesting.class,
                    IntakeTesting.class,
                    IntakeTuning.class,
                    PIDTuning.class,
                    RRPathTest.class,
                    ServoExTesting.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}*/
