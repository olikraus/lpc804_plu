

## lpc21isp

Sourcecode: https://github.com/olikraus/lpc21isp
Forked from: https://github.com/starblue/lpc21isp
Which was forked from: https://github.com/capiman/lpc21isp
Which was probably taken over from: https://sourceforge.net/projects/lpc21isp/files/lpc21isp/

lpc21isp features:
 - The version from starblue can flash LPC804 devices.
 - Compiles on linux and windows
 
LPC804: hex2lpc8xx vs lpc21isp 

| Property | hex2lpc8xx | lpc21isp |
|-------|---------|-------|
| Runs on Linux | yes | yes |
| Runs on Windows | no | yes |
| Device auto-detect | yes | yes |
| Flash speed (2K) | 5 seconds | < 1 second |
| Execute new code | yes (-x) | yes, but does not work in my case (why???) |
| Device not yet ready | Wait until it becomes ready | Abort with error |

Linux commandline:

```
./lpc21isp -verify lpc804_test.hex  /dev/ttyUSB0 115200 14746
```



