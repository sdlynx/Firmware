# Compiling NuttX

Compiling NuttX is easy if you already have the submodule placed properly. The "-j2" option specifies the number of cores your CPU has.

```bash
$ make -j2 archives
```

The NuttX build is placed in the Archives folder.

# Compiling Firmware

If you have not modified NuttX, you do not need to recompile the Archives every time. To compile the firmware for aerocore_default:

```bash
$ make -j2 aerocore_default
```

Also note on the [superior repository](https://github.com/sdlynx/LandYacht), there are scripts in the scripts directory to make the firmware. 