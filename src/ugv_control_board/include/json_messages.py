IMU_DATA_REQUEST = {"T": 126}
DISABLE_STREAM_CHASSIS_INFO = {"T": 131, "cmd": 0}
DISABLE_UART_WRITE_ECHO = {"T":143,"cmd":0}
ADD_ANY_BROADCAST_PEER = {"T": 303, "mac": "FF:FF:FF:FF:FF:FF"}

MSG_TEMPLATE = {"T": 306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":0,"h":0,"cmd":3,"megs":"<msg>"}
CMD_VEL_TEMPLATE = {"T": 13, "X": 0.0, "Z": 0.0}