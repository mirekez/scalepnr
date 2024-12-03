# scalepnr
Open source FPGA place and routing toolchain

# development
Currently development goes only under Windows MINGW-64 and Miniconda to be confident in Win64 building process.
Requires the following to be done:
 - Install msys2-x86_64-20240727.exe, Miniconda3-py39_24.7.1-0-Windows-x86_64.exe, run MSYS2 MSYS console
 - git clone https://github.com/mirekez/scalepnr; cd scalepnr
 - conda create -p ./.conda; source activate base; conda activate ./.conda; conda env update --file requirements.yaml
 - mkdir build; cd build; cmake -G "Unix Makefiles" ..; make

# license
This software is distributed under GPLv3, except libraries in folder libs/ which have their own Open-source licenses.
