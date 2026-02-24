package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PneumoConstants;

public class Pneumatics extends SubsystemBase {
  private final PneumaticHub pneumo = new PneumaticHub(PneumoConstants.HubId);

  // Left solenoid
  private final DoubleSolenoid leftSolenoid = pneumo.makeDoubleSolenoid(PneumoConstants.ForwardLeftSolenoidId, PneumoConstants.ReverseLeftSolenoidId);
  // Right solenoid
  private final DoubleSolenoid rightSolenoid = pneumo.makeDoubleSolenoid(PneumoConstants.ForwardRightSolenoidId, PneumoConstants.ReverseRightSolenoidId);

  public Pneumatics() {
  }

  public void setForward() {
    leftSolenoid.set(DoubleSolenoid.Value.kForward);
    rightSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setReverse() {
    leftSolenoid.set(DoubleSolenoid.Value.kReverse);
    rightSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setOff() {
    leftSolenoid.set(DoubleSolenoid.Value.kOff);
    rightSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void toggleCompressor() {
    if (pneumo.getCompressorCurrent() > 0.0)
      pneumo.disableCompressor();
    else
      pneumo.enableCompressorAnalog(PneumoConstants.MinPressure, PneumoConstants.MaxPressure);  
  }

  public void enableCompressor() {
    pneumo.enableCompressorAnalog(PneumoConstants.MinPressure, PneumoConstants.MaxPressure);  
  }

  public void disableCompressor() {
      pneumo.disableCompressor();
  }
}
