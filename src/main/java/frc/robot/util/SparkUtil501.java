// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import java.util.function.Supplier;

public class SparkUtil501 {
  /** Stores whether any error was has been detected by other utility methods. */
  public static boolean sparkStickyFault = false;
  /** */
  public static REVLibError sparkStickyError = REVLibError.kOk;

  /** Attempts to run the command until no error is produced. */
  public static REVLibError tryUntilOk(
      SparkBase spark, int maxAttempts, Supplier<REVLibError> command) {
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
        sparkStickyError = error;
      }
    }

    return error;
  }

  /** Attempts to run the command until no error is produced. */
  public static REVLibError tryUntilOk(
      RelativeEncoder encoder, int maxAttempts, Supplier<REVLibError> command) {
    REVLibError error = REVLibError.kOk;
    for (int i = 0; i < maxAttempts; i++) {
      error = command.get();
      if (error == REVLibError.kOk) {
        break;
      } else {
        sparkStickyFault = true;
        sparkStickyError = error;
      }
    }
    return error;
  }
}
