# Download and deploy yolov8 pose estimation model

Due to license issues, the current package doesn't provide a valid `network.c` and `network_ecblobs.h`. So to build and run the application, you first need to download and deploy the yolov8 pose estimation model. For that, follow these steps:

- [1. Download yolov8 pose estimation model](#1-download-yolov8-pose-estimation-model)
- [2. Generate C-Model from TFLite Model](#2-generate-c-model-from-tflite-model)
- [3. Program Your Network Data](#3-program-your-network-data)

## 1. Download yolov8 pose estimation model

Download the yolov8 pose estimation model using the following [link](https://github.com/stm32-hotspot/ultralytics/raw/refs/heads/main/examples/YOLOv8-STEdgeAI/stedgeai_models/pose_estimation/yolov8n_256_quant_pc_uf_pose_coco-st.tflite).

```
If You combine this software (“Software”) with other software from STMicroelectronics ("ST Software"), to generate a
software or software package ("Combined Software"), for instance for use in or in combination with STM32 products, You
must comply with the license terms under which ST distributed such ST Software ("ST Software Terms"). Since this
Software is provided to You under AGPL-3.0-only license terms, in most cases (such as, but not limited to, ST Software
delivered under the terms of SLA0044, SLA0048, or SLA0078), ST Software Terms contain restrictions which will strictly
forbid any distribution or non-internal use of the Combined Software. You are responsible for compliance with applicable
license terms for any Software You use, and as such, You must limit your use of this software and any Combined Software
accordingly.
```

## 2. Generate C-Model from TFLite Model

To generate the `network.c`, `network_ecblobs.h`, and the file containing network parameters, you must install STM32Cube.AI.

1. Add `<folderInstall>/Utilities/<your_os>/` to your path to have `stedgeai` known by your shell.
2. Add `<stm32cubeide_folderInstall>/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-<plugin_version>/tools/bin` to your path to have `arm-none-eabi-objcopy` known by your bash.

```bash
cd Model
stedgeai generate --no-inputs-allocation --no-outputs-allocation --model yolov8n_256_quant_pc_uf_pose_coco-st.tflite --target stm32n6 --st-neural-art default@user_neuralart.json
cp st_ai_output/network_ecblobs.h .
cp st_ai_output/network.c .
cp st_ai_output/network_atonbuf.xSPI2.raw network_data.xSPI2.bin
arm-none-eabi-objcopy -I binary network_data.xSPI2.bin --change-addresses 0x70380000 -O ihex network_data.hex
```

You can find the following script at [Model/generate-n6-model.sh](../Model/generate-n6-model.sh)

## 3. Program Your Network Data

Now you can program your network data in external flash.

```bash
export DKEL="<STM32CubeProgrammer_N6 Install Folder>/bin/ExternalLoader/MX66UW1G45G_STM32N6570-DK.stldr"

# Weights and parameters
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG ap=1 -el $DKEL -hardRst -w Model/network_data.hex
```
