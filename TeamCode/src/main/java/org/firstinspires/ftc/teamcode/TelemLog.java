package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class TelemLog {
    private final String BASE_FOLDER_NAME = "FIRST";

    private final Telemetry telemetry;
    private final Logger log;

    public TelemLog(Telemetry telemetry) {
        this.telemetry = telemetry;

        this.log = Logger.getLogger("TelemLog");
        setupLogFile();
    }

    private void setupLogFile() {
        try {
            // This block configure the logger with handler and formatter
            File directoryPath = Environment.getExternalStorageDirectory();

            FileHandler fh = new FileHandler(directoryPath.getPath() + "/TelemLog.log", true);
            this.log.addHandler(fh);

            SimpleFormatter formatter = new SimpleFormatter();
            fh.setFormatter(formatter);
        } catch (SecurityException | IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String msg, Object value, Level level) {
        this.telemetry.addData(msg, value);
        this.log.info(msg + value);
    }

    public void addData(String msg, Object value) {
        this.telemetry.addData(msg, value);
        this.log.info(msg + value);
    }

    public void addLine(String msg) {
        this.telemetry.addLine(msg);
        this.log.info(msg);
    }

    public void update() {
        this.telemetry.update();
    }
}
