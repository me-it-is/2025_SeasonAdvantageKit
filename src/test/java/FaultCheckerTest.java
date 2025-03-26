import static frc.robot.util.Elastic.Notification.NotificationLevel.ERROR;
import static frc.robot.util.Elastic.Notification.NotificationLevel.WARNING;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.sim.SparkSimFaultManager;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.faultChecker.UnitTestSparkFaultChecker;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class FaultCheckerTest {
  private static final int numOfFaultCombosToCheck = 1000;

  private record MeasageAndLevel(String mesage, NotificationLevel level) {}

  @Test
  public void automaticSparkMaxFaultCheckerTest() {
    final SparkMax testSparkMax = new SparkMax(0, MotorType.kBrushless);
    final SparkSimFaultManager sparkMaxFaultManager = new SparkSimFaultManager(testSparkMax);
    final UnitTestSparkFaultChecker sparkMaxChecker =
        new UnitTestSparkFaultChecker(testSparkMax, "SparkMax");

    automaticSparkFaultCheckerTester(sparkMaxChecker, sparkMaxFaultManager);
  }

  @Test
  public void automaticSparkFlexFaultCheckerTest() {
    final SparkFlex testSparkFlex = new SparkFlex(1, MotorType.kBrushless);
    final SparkSimFaultManager sparkFlexFaultManager = new SparkSimFaultManager(testSparkFlex);
    final UnitTestSparkFaultChecker sparkFlexChecker =
        new UnitTestSparkFaultChecker(testSparkFlex, "SparkFlex");

    automaticSparkFaultCheckerTester(sparkFlexChecker, sparkFlexFaultManager);
  }

  private void automaticSparkFaultCheckerTester(
      UnitTestSparkFaultChecker checker, SparkSimFaultManager faultManager) {
    List<MeasageAndLevel> lastExpectedFaultsAndWarnings = new ArrayList<>();
    List<MeasageAndLevel> lastExpectedFaults = new ArrayList<>();
    for (int i = 0; i < numOfFaultCombosToCheck; i++) {
      Faults faults = new Faults((int) Math.round(Math.random() * 0xff));
      faultManager.setFaults(faults);
      Warnings warnings = new Warnings((int) Math.round(Math.random() * 0xff));
      faultManager.setWarnings(warnings);

      List<MeasageAndLevel> expectedFaultsAndWarnings =
          getMesagesAndLevelsForFaultsAndWarnings(faults, warnings);
      List<MeasageAndLevel> expectedFaults =
          expectedFaultsAndWarnings.stream().filter((mAndL) -> (mAndL.level == ERROR)).toList();

      List<MeasageAndLevel> predictedNotifyedMeasages = new ArrayList<>();
      List<MeasageAndLevel> predictedLoggedMeasages = new ArrayList<>();

      for (MeasageAndLevel mAndL : expectedFaultsAndWarnings) {
        if (!lastExpectedFaultsAndWarnings.contains(mAndL)) {
          predictedLoggedMeasages.add(mAndL);
        }
      }

      for (MeasageAndLevel mAndL : lastExpectedFaultsAndWarnings) {
        if (!expectedFaultsAndWarnings.contains(mAndL)) {
          predictedLoggedMeasages.add(mAndL);
        }
      }

      for (MeasageAndLevel mAndL : expectedFaults) {
        if (!lastExpectedFaults.contains(mAndL)) {
          predictedNotifyedMeasages.add(mAndL);
        }
      }

      for (MeasageAndLevel mAndL : lastExpectedFaults) {
        if (!expectedFaults.contains(mAndL)) {
          predictedNotifyedMeasages.add(mAndL);
        }
      }

      checker.updateFaults();
      // creating a new list so i can sort it
      List<String> predictedNotifyedMesageStrings =
          new ArrayList<String>(
              predictedNotifyedMeasages.stream().map((mAndL) -> (mAndL.mesage)).toList());
      List<String> predictedLoggedMesageStrings =
          new ArrayList<String>(
              predictedLoggedMeasages.stream().map((mAndL) -> (mAndL.mesage)).toList());

      List<String> notifyedMesageStrings =
          new ArrayList<String>(checker.notifiedFaults.stream().map((f) -> (f.faultName)).toList());
      List<String> loggedMesageStrings =
          new ArrayList<String>(checker.loggedFaults.stream().map((f) -> (f.faultName)).toList());

      Collections.sort(predictedLoggedMesageStrings);
      Collections.sort(loggedMesageStrings);
      Collections.sort(predictedNotifyedMesageStrings);
      Collections.sort(notifyedMesageStrings);
      assertEquals(predictedLoggedMesageStrings, loggedMesageStrings);
      assertEquals(predictedLoggedMesageStrings, loggedMesageStrings);

      assertTrue(predictedNotifyedMesageStrings.containsAll(notifyedMesageStrings));
      assertTrue(notifyedMesageStrings.containsAll(predictedNotifyedMesageStrings));

      lastExpectedFaultsAndWarnings = expectedFaultsAndWarnings;
      lastExpectedFaults = expectedFaults;
    }
  }

  private List<MeasageAndLevel> getMesagesAndLevelsForFaultsAndWarnings(
      Faults faults, Warnings warnings) {
    return Stream.concat(
            convertFaultsObjectToMesagesAndLevels(faults).stream(),
            convertWarningsObjectToMesagesAndLevels(warnings).stream())
        .toList();
  }

  private List<MeasageAndLevel> convertFaultsObjectToMesagesAndLevels(Faults faults) {
    List<MeasageAndLevel> mesagesAndLevels = new ArrayList<>();
    if (faults.can) {
      mesagesAndLevels.add(new MeasageAndLevel("Can fault", ERROR));
    }
    if (faults.escEeprom) {
      mesagesAndLevels.add(new MeasageAndLevel("escEeprom fault", ERROR));
    }
    if (faults.firmware) {
      mesagesAndLevels.add(new MeasageAndLevel("Firmware fault", ERROR));
    }
    if (faults.gateDriver) {
      mesagesAndLevels.add(new MeasageAndLevel("Gate driver fault", ERROR));
    }
    if (faults.motorType) {
      mesagesAndLevels.add(new MeasageAndLevel("Incorect motor type", ERROR));
    }
    if (faults.other) {
      mesagesAndLevels.add(new MeasageAndLevel("Other fault", ERROR));
    }
    if (faults.sensor) {
      mesagesAndLevels.add(new MeasageAndLevel("Sensor fault", ERROR));
    }
    if (faults.temperature) {
      mesagesAndLevels.add(new MeasageAndLevel("Over temp", ERROR));
    }
    return mesagesAndLevels;
  }

  private List<MeasageAndLevel> convertWarningsObjectToMesagesAndLevels(Warnings warnings) {
    List<MeasageAndLevel> mesagesAndLevels = new ArrayList<>();
    if (warnings.brownout) {
      mesagesAndLevels.add(new MeasageAndLevel("Brownout detected", ERROR));
    }
    if (warnings.escEeprom) {
      mesagesAndLevels.add(new MeasageAndLevel("escEeprom warning", ERROR));
    }
    if (warnings.extEeprom) {
      mesagesAndLevels.add(new MeasageAndLevel("extEeprom warning", ERROR));
    }
    if (warnings.other) {
      mesagesAndLevels.add(new MeasageAndLevel("Other warning", ERROR));
    }
    if (warnings.overcurrent) {
      mesagesAndLevels.add(new MeasageAndLevel("Over current", ERROR));
    }
    if (warnings.sensor) {
      mesagesAndLevels.add(new MeasageAndLevel("Sensor warning", ERROR));
    }
    if (warnings.stall) {
      mesagesAndLevels.add(new MeasageAndLevel("Stall detected", WARNING));
    }
    if (warnings.hasReset) {
      mesagesAndLevels.add(new MeasageAndLevel("SparkMax reset", WARNING));
    }
    return mesagesAndLevels;
  }
}
