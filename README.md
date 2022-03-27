

https://user-images.githubusercontent.com/52521188/160302330-3fbaf9a3-48c4-42bb-9aa5-1210ab3598eb.mp4



# Physical School Experiments in Unity - Cable Simulation
All scripts can be found in `Assets/Scripts`. They rely on classes in `Assets/Lib`.

## Jakobsen's model
Jakobsen's model is implemented in `JakobsenCable.cs`. It implements the interface `Cable` defined in `Lib/Cable.cs`. It does not rely on other classes.

## (X)PBD
PBD & XPBD are implmented in `XPBDVerletCable.cs`. It also implements the interface `Cable` defined in `Lib/Cable.cs` and relies on the class `Constraint` defined in `Constraint.cs`.

## Link
This repository is accessible at https://gitlab.rlp.net/mamirago/bachelor-thesis.

