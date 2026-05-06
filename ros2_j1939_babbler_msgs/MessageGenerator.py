import cantools
import os
import sys
import re

# There are more rules than these, but these cover almost all realistic cases
# Additional restrictions I'm aware of:
#   - Field name can't start or end with underscore, can't have consecutive underscores
#   - Field names must start with lowercase letter
#   - Message name must start with uppercase letter
MESSAGE_NAME_MASK = re.compile('[^A-Za-z0-9]')
FIELD_NAME_MASK = re.compile('[^a-z0-9_]')

def convert_files(dbc_file, output_dir):
    print(f"Loading DBC file '{dbc_file}'")
    db = cantools.database.load_file(dbc_file)

    print(f"Exporting msg files to '{output_dir}'")
    os.makedirs(output_dir, exist_ok=True)  # Make the output directory if it doesn't exist

    # Loop through all the messages in the DBC and create ROS2 msg files for them
    for message in db.messages:
        print(f"Found message '{message.name}'")

        with open(f"{output_dir}/{MESSAGE_NAME_MASK.sub('', message.name)}.msg", "w") as f:
            # We always want messages to include a header (timestamp etc.) and source address
            f.write("std_msgs/Header header\n")
            f.write("uint8 src_addr\n\n")

            # Loop through each signal and add it into the ROS message type
            for signal in message.signals:
                dtype = None
                if signal.choices: # This is an enum
                    # Represent as the smallest uint type which can fit all the options
                    # Each option is added as a constant prefixed with the signal name
                    # Not sure if nested named values are legal, but if they are we won't support them
                    dtype = f"uint{ceil_bits(len(signal.choices))}"
                    for value, choice in signal.choices.items():
                        f.write(f"{dtype} {f'{signal.name}_{choice}'.upper()}={value}\n")
                elif signal.is_float:
                    # Use the length to decide if we need a float32 or float64, but note float8/float16 doesn't exist
                    dtype = f"float{ceil_bits(max(32, signal.length))}"
                elif signal.is_signed: # By default signal is either int/uint
                    dtype = f"int{ceil_bits(signal.length)}"
                else:  # Finally, if it isn't an enum, float, or signed int, it must be a uint
                    dtype = f"uint{ceil_bits(signal.length)}"
                f.write(f"{dtype} {FIELD_NAME_MASK.sub('', signal.name.lower())}\n\n")

def ceil_bits(bit_length):
    if bit_length <= 8: return 8
    if bit_length <= 16: return 16
    if bit_length <= 32: return 32
    if bit_length <= 64: return 64
    raise Exception("Signals with length greater than 64 bits are not supported")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise RuntimeError("Script must be called with a path to a DBC file and a path to export msg files to")
    convert_files(sys.argv[1], sys.argv[2])