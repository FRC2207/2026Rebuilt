// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;

/** Preset configurations for Spark Max periodic frame rates. */
public class SparkMaxPeriodicFrameConfig {
  public static void configNotLeader(SparkMax sparkMax, SparkMaxConfig config) {
    config.signals.primaryEncoderPositionPeriodMs(100);

    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configLeaderFollower(SparkMax sparkMax, SparkMaxConfig config) {
    config.signals.primaryEncoderPositionPeriodMs(10);

    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    // sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}