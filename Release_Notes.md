# Release Notes for x-cube-n6-ai-multi-pose-estimation Application

## Purpose

Computer Vision application to enable the deployment of object detections models on STM32N6570-DK board.

This application is prebuilt with a multi pose estimation model "YOLOv8 pose model".


## Key Features

- Multi-threaded application flow (FreeRTOS)
- Tracking and box/keypoints filtering
- NPU accelerated quantized AI model inference
- Dual DCMIPP pipes
- DCMIPP crop, decimation, downscale
- LTDC dual-layer implementation
- DCMIPP ISP usage
- Dev mode
- Boot from External Flash

## Software components

| Name                          | Version             | Release notes
|-----                          | -------             | -------------
| STM32Cube.AI runtime          | 11.0.0              | [release notes](Lib/AI_Runtime/README.md)
| Camera Middleware             | v1.5.0              | [release notes](Lib/Camera_Middleware/Release_Notes.md)
| lib_vision_models_pp Library  | v0.13.0             | [release notes](Lib/lib_vision_models_pp/lib_vision_models_pp/README.md)
| tracker                       | v1.0.1              | [release notes](Lib/tracker/Release_Notes.html)
| post process wrapper          | v1.0.9              | [release notes](Lib/ai-postprocessing-wrapper/Release_Notes.html)
| CMSIS                         | V6.2.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Documentation/index.html)
| STM32N6xx CMSIS Device        | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/CMSIS/Device/ST/STM32N6xx/Release_Notes.html)
| STM32N6xx HAL/LL Drivers      | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/STM32N6xx_HAL_Driver/Release_Notes.html)
| STM32N6570-DK BSP Drivers     | V1.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/STM32N6570-DK/Release_Notes.html)
| BSP Component aps256xx        | V1.0.6              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/aps256xx/Release_Notes.html)
| BSP Component Common          | V7.3.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/Common/Release_Notes.html)
| BSP Component mx66uw1g45g     | V1.1.0              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/mx66uw1g45g/Release_Notes.html)
| BSP Component rk050hr18       | V1.0.1              | [release notes](STM32Cube_FW_N6/Drivers/BSP/Components/rk050hr18/Release_Notes.html)
| FreeRTOS kernel               | v10.6.2             | [release notes](Lib/FreeRTOS/Source/History.txt)
| Fonts Utility                 | V2.0.3              | [release notes](STM32Cube_FW_N6/Utilities/Fonts/Release_Notes.html)
| lcd Utility                   | V2.2.0              | [release notes](STM32Cube_FW_N6/Utilities/lcd/Release_Notes.html)
| ai fsbl                       | v1.4.0              | [release notes](FSBL/Release_Notes.md)

## Update history

### v2.2.1 / September 2026

- Fix STM32_SigningTool_CLI API break since v2.21 by adding `--align` flag

### V2.2.0 / January 2026

- Update STEdgeAI to `v3.0.0` (incl. STM32Cube.AI runtime `v11.0.0`)
- Update Camera Middleware to `v1.5.0` adding support for VD1943 camera
- Update lib_vision_models_pp to `v0.13.0`
- Update post process wrapper to `v1.0.9`
- Update STM32Cube_FW_N6 to `V1.3.0`
- Provide single, combined prebuilt binary instead of three binary files to
  simplify programming procedure.

### V2.1.0 / August 2025

- Update STEdgeAI to `v2.2.0` (incl. STM32Cube.AI runtime `v10.2.0`)
- Update lib_vision_models_pp to `v0.10.0`
- Update post process wrapper to `v1.0.7`
- Update Camera Middleware to `v1.4.3`
- Update STM32Cube_FW_N6 to `V1.2.0`

### V2.0.0 / May 2025

- Add tracking and filtering
- Replace Threadx by FreeRTOS

### V1.0.0 / December 2024

Initial Version
