The directories contains all the necessary configuration files. The ".ioc" files can be opened using CubeIDE or CubeMX, all the pins/clocks/peripherals are already ready to use.

Code was tested using STM32L4 + RFM95.

## Directories
- lora_code contains basic code for bare lora. The main.c presents an example with transmission events scheduled using LPTIM timers.
- lora_hop_code contains the code for the complete application (node sequence)
- test_code is the code for a nucleo board used for injecting test messages in the lora_hop sequence of nodes.
- BC_lora_hop_test contains "test_code" code designed for a custom node (i.e. not valid for Nucleo Boards).
- lora_hop_head contains code for the NODE-0 -> It simply schedules TX acknowledgments for other nodes and prints on serial all the received payloads (Connect a laptop to analyze all the data).
- pythonCode contains code used to read and parse serial connected to "lora_hop_head"-NODE.






