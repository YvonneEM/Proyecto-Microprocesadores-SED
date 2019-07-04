SET PATH=C:\Keil_v5\ARM\ARMCC\Bin;C:\Program Files (x86)\Common Files\Oracle\Java\javapath;C:\Program Files (x86)\Intel\iCLS Client\;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files (x86)\Intel\OpenCL SDK\2.0\bin\x86;C:\Program Files (x86)\Intel\OpenCL SDK\2.0\bin\x64;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files (x86)\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files\MATLAB\R2017b\runtime\win64;C:\Program Files\MATLAB\R2017b\bin;C:\Program Files\MATLAB\R2010a\runtime\win64;C:\Program Files\MATLAB\R2010a\bin;C:\Modeltech_pe_edu_10.2c\win32pe_edu
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F411VETx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Prj_M
SET CPU_CLOCK=0x017D7840
"C:\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via "..\..\..\..\micros-control de termostato\startup_stm32f411xe._ia"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\main.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_it.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_msp.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_adc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_adc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_tim.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_tim_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_rcc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_rcc_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_flash.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_flash_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_flash_ramfunc.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_gpio.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_dma_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_dma.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_pwr.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_pwr_ex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal_cortex.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\stm32f4xx_hal.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via "..\..\..\..\micros-control de termostato\system_stm32f4xx.__i"
"C:\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via "..\..\..\..\Micros-Control de Termostato\Control de Termostato.lnp"
"C:\Keil_v5\ARM\ARMCC\Bin\fromelf.exe" "..\..\..\..\Micros-Control de Termostato\Control de Termostato.axf" --i32combined --output "..\..\..\..\Micros-Control de Termostato\Control de Termostato.hex"
