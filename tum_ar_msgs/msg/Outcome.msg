# ID of the outcome state
uint8 id
# Name of the outcome state
string name
# Type of the outcome state.
uint8 type

# Possible outcome types
uint8 TYPE_DEFAULT = 0
uint8 TYPE_INFO = 1
uint8 TYPE_SUCCESS = 2
uint8 TYPE_WARN = 3
uint8 TYPE_ERROR = 4

# Reserved outcome IDs
uint8 ID_TIMEOUT = 254
uint8 ID_TASK_ABORTED = 255