package org.firstinspires.ftc.teamcode.logging;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.List;

public class Logger {

    private final Telemetry telemetry;
    private final List<String> errorLog = new ArrayList<>();
    private final List<String> constantVal = new ArrayList<>();

    private static final int MAX_LOG_LINES = 5;

    public Logger(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void log(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void logError(String error) {
        errorLog.add(0, "[ERROR] " + error);

        while (errorLog.size() > MAX_LOG_LINES) {
            errorLog.remove(errorLog.size() - 1);
        }
    }

    public void update() {
        for (String line : constantVal) {
            telemetry.addLine(line);
        }

        telemetry.addLine("---- Log ----");
        for (String line : errorLog) {
            telemetry.addLine(line);
        }
        telemetry.addLine("-------------");
        telemetry.update();
    }

    public void clear() {
        telemetry.clear();
        errorLog.clear();
    }

    public void logValue(String input, int index) {
        while (constantVal.size() <= index) {
            constantVal.add("");
        }
        constantVal.set(index, input);
    }
}
