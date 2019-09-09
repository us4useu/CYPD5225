@echo "Running post build commands"
@echo "---------------------------"

@set "PROJ=CYPD5125-40LQXI_notebook"
@set "CONFIG=Debug"
if "%1" == "Release" (
    @set "CONFIG=Release"
)

move CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%.cyacd CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%_2.cyacd
copy CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%.hex CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%_2.hex

copy backup_fw.cydsn\CortexM0\ARM_GCC_541\%CONFIG%\backup_fw_1.cyacd CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%_1.cyacd
copy backup_fw.cydsn\CortexM0\ARM_GCC_541\%CONFIG%\backup_fw_1.hex CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%_1.hex

cyelftool.exe -M CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%.elf backup_fw.cydsn\CortexM0\ARM_GCC_541\%CONFIG%\backup_fw_1.elf CortexM0\ARM_GCC_541\%CONFIG%\%PROJ%.hex --flash_row_size 256 --flash_size 131072 --flash_array_size 131072

@echo "---------------------------"
@echo "Done"
