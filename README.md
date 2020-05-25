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

