// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HealthChecker;
import java.util.Queue;
import java.util.Vector;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.Elastic;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO, HealthChecker {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    pigeon.getAccumGyroX();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.xRotation = pigeon.getAccumGyroX().getValue();
    inputs.xAngularVelocity = pigeon.getAngularVelocityXDevice().getValue();

    inputs.yRotation = pigeon.getAccumGyroY().getValue();
    inputs.yAngularVelocity = pigeon.getAngularVelocityYDevice().getValue();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public boolean checkHealth() {
    pigeonFaults faults = new pigeonFaults();
    faults.updateFaults();
    if (faults.getFaults().size() != 0) {
      String allErrors = "";
      for (String s : faults.getFaults())
      {
        allErrors += s;
        allErrors += " Fault, ";
      }
      // to remove the final ", "
      allErrors.substring(0, allErrors.lastIndexOf(","));
      
      Notification badGryo = new Notification(NotificationLevel.WARNING, "Gryo error", allErrors);
      Elastic.sendNotification(badGryo);
    }
    return faults.getFaults().size() == 0;
  }


  public void addNotifacationToElastic() {
    // TODO Auto-generated method stub
    
  }

  public class pigeonFaults {
    public boolean bootDurringEnable = false;
    public boolean bootIntoMotion = false;
    public boolean bootUpAccelerometer = false;
    public boolean bootUpGyroscope = false;
    public boolean bootUpMagnetometer = false;
    public boolean dataAquiredLate = false;
    public boolean hardware = false;
    public boolean loopTimeSlow = false;
    public boolean saturatedAccelerometer = false;
    public boolean saturatedGyroscope = false;
    public boolean saturatedMagnetometer = false;
    public boolean underVoltage = false;
    public boolean unlicensedFeatureInUse = false;

    public void updateFaults() {
      bootDurringEnable = pigeon.getFault_BootDuringEnable(true).getValue();
      bootIntoMotion = pigeon.getFault_BootIntoMotion(true).getValue();
      bootUpAccelerometer = pigeon.getFault_BootupAccelerometer(true).getValue();
      bootUpGyroscope = pigeon.getFault_BootupGyroscope(true).getValue();
      bootUpMagnetometer = pigeon.getFault_BootupMagnetometer(true).getValue();
      dataAquiredLate = pigeon.getFault_DataAcquiredLate(true).getValue();
      hardware = pigeon.getFault_Hardware(true).getValue();
      loopTimeSlow = pigeon.getFault_LoopTimeSlow(true).getValue();
      saturatedAccelerometer = pigeon.getFault_SaturatedAccelerometer(true).getValue();
      saturatedGyroscope = pigeon.getFault_SaturatedGyroscope(true).getValue();
      saturatedMagnetometer = pigeon.getFault_SaturatedMagnetometer(true).getValue();
      underVoltage = pigeon.getFault_Undervoltage(true).getValue();
      unlicensedFeatureInUse = pigeon.getFault_UnlicensedFeatureInUse(true).getValue();
    }

    public Vector<String> getFaults() {
      Vector<String> faults = new Vector<String>();
      
      if(bootDurringEnable) { faults.add("BootDurringEnable");}
      if(bootIntoMotion) { faults.add("BootIntoMotion");}
      if(bootUpAccelerometer) { faults.add("BootUpAccelerometer");}
      if(bootUpGyroscope) { faults.add("BootUp Gryoscope");}
      if(bootUpMagnetometer) { faults.add("BootUpMagnetomter");}
      if(dataAquiredLate) { faults.add("DataAquiredLate");}
      if(hardware) { faults.add("Hardware");}
      if(loopTimeSlow) { faults.add("LoopTimeSlow");}
      if(saturatedAccelerometer) { faults.add("SaturatedAccelerometer");}
      if(saturatedGyroscope) { faults.add("SaturatedGyroscope");}
      if(saturatedMagnetometer) { faults.add("saturatedMagnetometer");}
      if(underVoltage) { faults.add("UnderVoltage");}
      if(unlicensedFeatureInUse) { faults.add("UnlicensedFeatureInUse");}

      return faults;
    }
  }
}
