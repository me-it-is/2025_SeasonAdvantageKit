package frc.robot.util.faultChecker;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.FaultConstants;

public class CTREFaultChecker extends AbstractFaultChecker {
  public CTREFaultChecker(String deviceName) {
    super(deviceName);
  }

  public CTREFaultChecker(TalonFX device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getTalonFXFaults(device));
  }

  public CTREFaultChecker(Pigeon2 device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getPigeonFaults(device));
  }

  public CTREFaultChecker(CANcoder device, String deviceName) {
    super(deviceName);
    super.addFaults(FaultConstants.getCANCoderFaults(device));
  }
}
