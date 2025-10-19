import os
import time
from ainex_sdk.gpio import *

def get_button_status():
    return get_gpio(adapter_board_key)

