### Automation Lab, Sungkyunkwan University

#### GitHub Stats
![](https://img.shields.io/github/downloads/SKKU-AutoLab-VSW/ETSS-08-Data/total.svg?style=for-the-badge)


# Traffic Surveillance Dataset
Traffic Surveillance Data Generation capable of producing various environment record on road by using Carla.

![gif](images/traffic_surveillance_intersection.gif)

### I. Building CARLA
Use `git clone` or download the project from [CARLA Github][carlagithublink].

Then follow the instruction at [How to build on Linux][buildlinuxlink] or [How to build on Windows][buildwindowslink].

The Linux build needs for an UE patch to solve some visualization issues regarding Vulkan. Those already working with a Linux build should install the patch and make the UE build again using the following commands.  
```sh
# Download and install the UE patch  
cd ~/UnrealEngine_4.24
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/UE_Patch/430667-13636743-patch.txt ~/430667-13636743-patch.txt
patch --strip=4 < ~/430667-13636743-patch.txt
# Build UE
./Setup.sh && ./GenerateProjectFiles.sh && make
```

[carlagithublink]: https://github.com/carla-simulator/carla
[buildlinuxlink]: https://carla.readthedocs.io/en/latest/build_linux/
[buildwindowslink]: https://carla.readthedocs.io/en/latest/build_windows/

### II. Instruction
Please refer to [INSTRUCTION.md](/Instruction.md) for how to use.

### III. Sample
Please go to this repository for [Realistic-Traffic-Surveillance Generated Sample](https://github.com/SKKU-AutoLab-VSW/Realistic-Traffic-Surveillance_GeneratedSample)
