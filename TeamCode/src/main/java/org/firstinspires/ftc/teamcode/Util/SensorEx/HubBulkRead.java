package org.firstinspires.ftc.teamcode.Util.SensorEx;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.Generals.Enums;

import java.util.List;

public class HubBulkRead {
    public static LynxModule CONTROL_HUB, EXPANSION_HUB;
    private List<LynxModule> allHubs;

    private static HubBulkRead instance = null;
    private boolean justControlHub = false;

    private LynxModule.BulkCachingMode currentCachingMode = LynxModule.BulkCachingMode.MANUAL;

    public static HubBulkRead getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new HubBulkRead(hardwareMap);
        }

        return instance;
    }

    public static HubBulkRead getInstance(HardwareMap hardwareMap, LynxModule.BulkCachingMode cachingMode) {
        if (instance == null) {
            instance = new HubBulkRead(hardwareMap, cachingMode);
        }

        return instance;
    }

    public HubBulkRead(HardwareMap hardwareMap){
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);

        init(currentCachingMode);

        CONTROL_HUB = allHubs.get(0);
        if (allHubs.size() > 1) { EXPANSION_HUB = allHubs.get(1); }
            else { justControlHub = true; }
    }

    public HubBulkRead(HardwareMap hardwareMap, LynxModule.BulkCachingMode cachingMode){
        allHubs = hardwareMap.getAll(LynxModule.class);
        this.currentCachingMode = cachingMode;

        init(currentCachingMode);

        CONTROL_HUB = allHubs.get(0);
        if (allHubs.size() > 1) { EXPANSION_HUB = allHubs.get(1); }
    }

    private void init(LynxModule.BulkCachingMode cachingMode) {
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(cachingMode);
        }
    }

    synchronized public void clearCache(Enums.Hubs type) {
        switch (type) {
            case CONTROL_HUB: { CONTROL_HUB.clearBulkCache(); } break;
            case EXPANSION_HUB: { if (!justControlHub) { EXPANSION_HUB.clearBulkCache(); } } break;
            case ALL: {
                for(LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
            } break;
            default: {}
        }
    }

    public LynxModule.BulkCachingMode getCurrentCachingMode() { return currentCachingMode; }
}
