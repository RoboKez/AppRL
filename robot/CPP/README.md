Notes if you want a fresh xcode build

1) In the xcode side bar copy folder <CPP> amd <Externals> to your SwiftUI project with create copy selected (e.g. to RL.xcodepj). Replace Eigen and MuJoCo framework with the latest version avalible if desired


2) Add header search paths for:
a) MuJoCo:  <$(PROJECT_DIR)/Externals/Frameworks/MuJoCo.framework/Headers>
b) Eigen:   <$(PROJECT_DIR)/Externals/Include/Eigen>  //Note that Eigen Folder is in a Eigen Folder

3) Add framework search path for Mujoco: <$(PROJECT_DIR)/Externals/Frameworks>

3) Build Phases > Link Binaries with Libraries > add MuJoCo.framework from you External/Framework folder

4) As of writting this Mujoco is currently at version 2.1.3 and uses a dynamic library. Therefore, go to General > select Frameworks, Libraries, and Embedded Content. Under Embed change <Do Not Embed> to <Embed without signing> for MuJoCo.Framework (This should copy it over in the build stage) and Framework should already be signed by DeepMind. TODO: Check if there is an alternative approach as MuJoCo still needs referencing however deleting Apple Developer isn't great. Delete any other files in this table

5) As this is using the dynamic MuJoCo library which is not in /usr/lib <Code Signing Entitlements> and <Code signing Identity> need removing under build settings.

6) Build Phases > Compile Source and add all the c and cpp files in Externals. Dont forget mujoco's uitools.c in framework examples.

It should run now you might get 56 comment warnings from Eigen. If you cant find rlbyte symbols make sure you header bridge is linked in settings



$(PROJECT_DIR)/Externals/Libraries
