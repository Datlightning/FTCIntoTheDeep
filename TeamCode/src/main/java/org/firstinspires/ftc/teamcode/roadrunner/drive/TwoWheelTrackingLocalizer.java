package org.firstinspires.ftc.teamcode.roadrunner.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {

    public static double FORWARD_OFFSET = 0;
    public static double HORIZONTAL_OFFSET = 0;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 1; // X is the up and down direction
    public static double PARALLEL_Y =  -2; // Y is the strafe direction

    public static double PERPENDICULAR_X = 3.4;
    public static double PERPENDICULAR_Y = .5;

    public static double X_MULTIPLIER = 1.053076152643298; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.045916600029769; // Multiplier in the Y direction

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;
    private RobotConstants mc;
    private MecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, MecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y + FORWARD_OFFSET, 0),
                new Pose2d(PERPENDICULAR_X + HORIZONTAL_OFFSET, PERPENDICULAR_Y , Math.toRadians(90))
        ));


        this.drive = drive;
        mc = new RobotConstants();
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, mc.bl));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, mc.fr));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        parallelEncoder.setDirection(Encoder.Direction.FORWARD);
        perpendicularEncoder.setDirection(Encoder.Direction.FORWARD);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}