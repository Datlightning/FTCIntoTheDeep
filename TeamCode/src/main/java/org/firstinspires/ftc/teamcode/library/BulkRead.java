package org.firstinspires.ftc.teamcode.library;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

import kotlin.text.CharDirectionality;

public class BulkRead {
    private List<LynxModule> allHubs;
    public BulkRead(HardwareMap hardwareMap){
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public void clearCache(){
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}

