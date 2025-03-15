package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.testing.AutoArmStabilizerTesting;
import org.firstinspires.ftc.teamcode.testing.BandedSlidesSystem;
import org.firstinspires.ftc.teamcode.testing.BigBoyTesting;
import org.firstinspires.ftc.teamcode.testing.CameraTesting;
import org.firstinspires.ftc.teamcode.testing.ColorTesting;
import org.firstinspires.ftc.teamcode.testing.DiffyIntakeTest;
import org.firstinspires.ftc.teamcode.testing.DistanceRRPathTest;
import org.firstinspires.ftc.teamcode.testing.DoubleTap;
import org.firstinspires.ftc.teamcode.testing.DriveTesting;
import org.firstinspires.ftc.teamcode.testing.EpicWebTransferTest;
import org.firstinspires.ftc.teamcode.testing.IntakeTuning;
import org.firstinspires.ftc.teamcode.testing.MotionProfileTest;
import org.firstinspires.ftc.teamcode.testing.PIDTuning;
import org.firstinspires.ftc.teamcode.testing.PoseTrasnferTest;
import org.firstinspires.ftc.teamcode.testing.PotTesting;
import org.firstinspires.ftc.teamcode.testing.RRPathTest;
import org.firstinspires.ftc.teamcode.testing.RudimentaryDiffyTesting;
import org.firstinspires.ftc.teamcode.testing.ServoExTesting;
import org.firstinspires.ftc.teamcode.testing.ServoTesting;
import org.firstinspires.ftc.teamcode.testing.SlideTesting;
import org.firstinspires.ftc.teamcode.testing.TestDataTrasnfer;
import org.firstinspires.ftc.teamcode.testing.TrafficLightTest;

import java.util.Arrays;

public final class TestingOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "testing";
    public static final boolean DISABLED = true;

    private TestingOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP )
                .build();
    }
    private static OpModeMeta metaForClass(Class<? extends OpMode> cls, boolean teleop) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(teleop ? OpModeMeta.Flavor.TELEOP : OpModeMeta.Flavor.AUTONOMOUS)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        manager.register(metaForClass(AutoArmStabilizerTesting.class, false), AutoArmStabilizerTesting.class);
        manager.register(metaForClass(CameraTesting.class), CameraTesting.class);
        manager.register(metaForClass(DiffyIntakeTest.class), DiffyIntakeTest.class);
        manager.register(metaForClass(DistanceRRPathTest.class, false), DistanceRRPathTest.class);
        manager.register(metaForClass(DoubleTap.class), DoubleTap.class);
        manager.register(metaForClass(DriveTesting.class), DriveTesting.class);
        manager.register(metaForClass(EpicWebTransferTest.class), EpicWebTransferTest.class);
        manager.register(metaForClass(IntakeTuning.class), IntakeTuning.class);
        manager.register(metaForClass(MotionProfileTest.class), MotionProfileTest.class);
        manager.register(metaForClass(PIDTuning.class), PIDTuning.class);
        manager.register(metaForClass(PoseTrasnferTest.class), PoseTrasnferTest.class);
        manager.register(metaForClass(RRPathTest.class, false), RRPathTest.class);
        manager.register(metaForClass(RudimentaryDiffyTesting.class), RudimentaryDiffyTesting.class);
        manager.register(metaForClass(ServoExTesting.class), ServoExTesting.class);
        manager.register(metaForClass(ServoTesting.class), ServoTesting.class);
        manager.register(metaForClass(SlideTesting.class), SlideTesting.class);
        manager.register(metaForClass(TestDataTrasnfer.class, false), TestDataTrasnfer.class);
        manager.register(metaForClass(TrafficLightTest.class), TrafficLightTest.class);
        manager.register(metaForClass(ColorTesting.class), ColorTesting.class);
        manager.register(metaForClass(PotTesting.class), PotTesting.class);
        manager.register(metaForClass(BandedSlidesSystem.class), BandedSlidesSystem.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AutoArmStabilizerTesting.class,
                    CameraTesting.class,
                    ColorTesting.class,
                    DiffyIntakeTest.class,
                    DistanceRRPathTest.class,
                    DoubleTap.class,
                    DriveTesting.class,
                    EpicWebTransferTest.class,
                    IntakeTuning.class,
                    MotionProfileTest.class,
                    PIDTuning.class,
                    PoseTrasnferTest.class,
                    RRPathTest.class,
                    RudimentaryDiffyTesting.class,
                    ServoExTesting.class,
                    ServoTesting.class,
                    SlideTesting.class,
                    PotTesting.class,
                    TestDataTrasnfer.class,
                    TrafficLightTest.class,
                    BandedSlidesSystem.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}