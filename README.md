# mgear_solvers_rumba

Port of the mgear_solvers project to Rumba

## Build

To build it on Windows, you need a Visual Studio 2015 and a Rumba installation which contains a SDK.

```
> cd mgear_solvers_rumba
> mkdir build
> cd build
> cmake ../src -G "Visual Studio 14 2015 Win64" "-DRUMBA_SDK=/c/Program Files/Rumba/sdk/"
> cmake --build .
```

## Installation

### Declare the plug-ins to mtorba

First, in order to export your rig from Maya, you have to declare your plug-ins to MTORBA.

To do that, add to the MTORBA_USER_EXPORTERS environment variable the path to the mgear_solvers_rumba/mtorba directory:

```MTORBA_USER_EXPORTERS=C:/mgear_solvers_rumba/mtorba/```

If you have multiple mtorba directories (maybe for weightDriver..) use your OS separator character, ';' for Windows, ':' for Linux.

```MTORBA_USER_EXPORTERS=C:/mgear_solvers_rumba/mtorba/;C:/weightDriver_rumba/mtorba/```

Now you should be able to export your rig.

### Register the plug-ins in Rumba

Now, you want to register your plug-ins to Rumba.

To do that, add the directory plugin path to the RUMBA_USER_PLUGINS environment variable:

```RUMBA_USER_PLUGINS=C:/mgear_solvers_rumba/build/release"```

If you have multiple plug-ins directories (maybe for weightDriver..) use your OS separator character, ';' for Windows, ':' for Linux.

```RUMBA_USER_PLUGINS=C:/mgear_solvers_rumba/build/release/;C:/weightDriver_rumba/build/release/```
