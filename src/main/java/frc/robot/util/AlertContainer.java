package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class AlertContainer {
    public List<Alert> alerts = new ArrayList<>();
    
    public Alert register(Alert alert) {
        alerts.add(alert);
        return alert;
    }

    public void clearAll() {
        for(Alert alert : alerts) {
            alert.set(false);
        }
    }

    public List<Alert> getMajorFaults() {
        List<Alert> majorAlerts = new ArrayList<>();
        for(Alert alert : alerts) {
            if(alert.getType() == AlertType.kError && alert.get()) {
                majorAlerts.add(alert);
            }
        }
        return majorAlerts;
    }

    public List<Alert> getMinorFaults() {
        List<Alert> minorAlerts = new ArrayList<>();
        for(Alert alert : alerts) {
            if(alert.getType() == AlertType.kWarning && alert.get()) {
                minorAlerts.add(alert);
            }
        }
        return minorAlerts;
    }

    private static AlertContainer _instance;
    public static AlertContainer getInstance() {
        if(_instance == null) _instance = new AlertContainer();
        return _instance;
    }
}