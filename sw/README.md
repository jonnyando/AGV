# Software

to setup toolchain, download latest [arm-non-eabi-gcc package](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)

```bash
$ cd "${HOME}"/opt
$ tar xjf ~/Downloads/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
$ chmod -R -w "${HOME}"/opt/gcc-arm-none-eabi-7-2017-q4-major
```

Update Makefile with __BINPATH__ reflecting the */opt/gcc-arm-none-eabi-7-2017-q4-major/bin/* folder created



vscode:
visual theme : `atom one dark`
icon theme : `material icon theme`