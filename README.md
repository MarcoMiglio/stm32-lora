The directories contains all the necessary configuration files. The ".ioc" files can be opened using CubeIDE or CubeMX, all the pins/clocks/peripherals are already ready to use.

Code was tested using STM32L4 + RFM95.

## Directories
- lora_code contains basic code for bare lora. The main.c presents an example with transmission events scheduled using LPTIM timers.
- lora_hop_code contains the code for the complete application (node sequence)
- test_code is the code for a nucleo board used for injecting test messages in the lora_hop sequence of nodes.







