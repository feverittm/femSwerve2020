/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Logger extends SubsystemBase {
  private static Logger instance;
  private BufferedWriter writer;
  private boolean logging = true;
  private long startTime = 0;
  DriverStation ds;
  private File path;
  private static final String SD_FILE_NAME = "File Name: ";
  private static final String LOGSTRING = "Logging";
  private File basedir = new File("/home/lvuser/spartanlogs");

  public Logger() {
    this.ds = DriverStation.getInstance();
    SmartDashboard.putBoolean(LOGSTRING, logging);
    logging = wantToLog();
    if (logging) {
      SmartDashboard.putString(SD_FILE_NAME, "");
      path = openFile();
      writeFileHeader(path);
    }
  }

  /**
   * @return File
   */
  private File openFile() {
    checkPath(basedir);
    return getPath(basedir, getFileIndex(basedir));
  }

  /**
   * @param path
   */
  private void checkPath(File path) {
    /* check for base directory */
    if (!path.exists()) {
      path.mkdir();
    }
  }

  /**
   * @param path
   * @return int
   */
  private int getFileIndex(File path) {
    int max = 0;
    File[] files = path.listFiles();
    if (files != null) {
      for (File file : files) {
        if (file.isFile()) {
          try {
            int index = Integer.parseInt(file.getName().split("_")[0]);
            if (index > max) {
              max = index;
              max++;
            }
          } catch (Exception e) {
            e.printStackTrace();
          }
        }
      }
    }
    return max;
  }

  /**
   * @return String
   */
  private File getPath(File path, int index) {
    String sdFileName = SmartDashboard.getString(SD_FILE_NAME, "NO NAME");
    String fs;

    if (ds.isFMSAttached()) {
      fs = String.format("%d_%s_%d_log.txt", index, ds.getAlliance().name(), ds.getLocation());
    } else if (sdFileName != null) {
      fs = String.format("%d_%s.txt", index, sdFileName);
    } else {
      fs = String.format("%d_log.txt", index);
    }
    return new File(path, fs);
  }

  /**
   * Write a CSV header into the log file.
   * 
   * @param path
   */
  public void writeFileHeader(File path) {
    if ((wantToLog() || ds.isFMSAttached())) {
      try {
        writer = new BufferedWriter(new FileWriter(path));
        writer.write("time,rpms,ballspeed,setpoint,output");
        writer.newLine();
        startTime = System.currentTimeMillis();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Write a timestamped entry in the log file.
   */
  public void logAll() {
    String logString = "%.3f";
    double timeSinceStart;

    if (this.wantToLog()) {
      try {
        timeSinceStart = (System.currentTimeMillis() - this.startTime) / 1000.0;
        this.writer.write(String.format(logString, timeSinceStart));

        writer.write(String.format(logString, ShooterSubsystem.getInstance().getRPMs()));
        writer.write(String.format(logString, ShooterSubsystem.getInstance().getBallSpeed()));
        writer.write(String.format(logString, ShooterSubsystem.getInstance().getSetpoint()));
        writer.write(String.format(logString, ShooterSubsystem.getInstance().getOutput()));
        writer.newLine();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * getter to grab the logging status from the smartdashboard entry
   * 
   * @return boolean
   */
  public boolean wantToLog() {
    logging = SmartDashboard.getBoolean(LOGSTRING, true);
    return logging;
  }

  /**
   * Simple clean file close method
   * 
   * @return boolean
   */
  public void close() {
    if (writer != null && wantToLog()) {
      try {
        this.writer.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * @return Logger
   */
  public static Logger getInstance() {
    if (instance == null)
      instance = new Logger();
    return instance;
  }
}