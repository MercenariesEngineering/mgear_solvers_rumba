# mgear_solvers_rumba

Port of the mgear_solvers project to Rumba

The official MGear web site : https://www.mgear-framework.com/

This plug-in is included in the Rumba distribution starting from version 1.0.rc-3.

## Build

To build it on Windows, you need a Visual Studio 2015 and a Rumba installation which contains a SDK.

```
> cd mgear_solvers_rumba
> mkdir build
> cd build
> cmake ../src -G "Visual Studio 14 2015 Win64" "-Drumba_DIR=/c/Program Files/Rumba/sdk/"
> cmake --build . --config release
```

## Installation

### Register the plug-ins in Rumba

Now, you want to register your plug-ins to Rumba.

To do that, add the directory plugin path to the RUMBA_USER_PLUGINS environment variable:

```RUMBA_USER_PLUGINS=C:/mgear_solvers_rumba/build/release"```

If you have multiple plug-ins directories (maybe for weightDriver..) use your OS separator character, ';' for Windows, ':' for Linux.

```RUMBA_USER_PLUGINS=C:/mgear_solvers_rumba/build/release/;C:/weightDriver_rumba/build/release/```
