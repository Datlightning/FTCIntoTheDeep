package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.library.Subsystem;
import org.firstinspires.ftc.teamcode.testing.BigBoyTesting;
import org.firstinspires.ftc.teamcode.vision.TheBigBrainAlgorithm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;


public class Camera extends Subsystem {

    OpenCvCamera camera;
    TheBigBrainAlgorithm bigBrainAlgorithm;
    Telemetry telemetry;

    private boolean on = true;
    private boolean inside_pick = true;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry){
        bigBrainAlgorithm = new TheBigBrainAlgorithm();
        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, RobotConstants.camera), cameraMonitorViewId);


    }
    public void useInsidePick(boolean on){
        inside_pick = on;
        bigBrainAlgorithm.useInsidePick(inside_pick);
    }

    public void startCamera(){
        on = true;
        camera.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(camera, 5);
    }
    public boolean isCameraOn(){
        return on;
    }
    public void stopCamera(){
        on = false;
        camera.stopStreaming();
    }
    public double[] getRed(){
        return bigBrainAlgorithm.redPosition();
    }
    public double[] getBlue(){
        return bigBrainAlgorithm.bluePosition();
    }
    public double[] getYellow(){
        return bigBrainAlgorithm.yellowPosition();
    }
    @Override
    public void update() {

    }

    @Override
    public void telemetry() {
        telemetry.addData("Red Position", toString(getRed()));
        telemetry.addData("Blue Position", toString(getBlue()));
        telemetry.addData("Yellow Position", toString(getYellow()));
    }
    public String toString(double[] array){
        double angle = array[0];
        double lateral = array[1];
        double linear = array[2];
        return "Degree: " + angle + ", Lateral: " + lateral + ", Linear: " + linear;

    }
    @Override
    public void init() {
        camera.setPipeline(bigBrainAlgorithm);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 5);

            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }
}
