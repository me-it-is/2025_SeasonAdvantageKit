# 2025_Season

[download WPILib](github.com/wpilibsuite/allwpilib/releases) and then run the installer, then clone this repo with `git clone https://github.com/me-it-is/2025_SeasonAdvantageKit.git` then run open the folder in whatever and run ./gradlew SimulateJava if you get a lot of loop overrun messages then you might need to set the camera fps to zero in [VisionIOSim.java](src/main/java/frc/robot/subsystems/vision/VisionIOSim.java#L33), if you then still have loop overuns you can try setting kdt to 0.01 rather than 0.005 in [Constants.java](src/main/java/frc/robot/Constants.java#L83). Then open advantage scope and use ctrl+shift+k or command+shift+k on mac to connect to the simulation, finally follow the steps bellow to add just tod to the field

Our code for the 2025 season.
[Version Control Protocol](https://docs.google.com/document/d/10Hep8I_G-WECgAwW-7c-CxzomMbJLIjcVaO4hX_GPtE/edit?tab=t.0#heading=h.aukg37ez4vyu)
[Clickup Tasks](https://app.clickup.com/9014321115/v/b/6-901403101610-2)

Set the advantagescope to use the assets folder by app or help(depends on the version) and then Use custom assets folder and selecting the assets folder. You can then set the robot display to `just tod` and drag the MechanismLocations on to the position to add the components.

for more info see the ascope [docs](https://docs.advantagescope.org/more-features/custom-assets)
