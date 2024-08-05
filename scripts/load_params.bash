#!/bin/bash

# Check if any arguments were provided
if [ $# -eq 0 ]; then
    echo "Please provide one or more configuration file names as input."
    exit 1
fi

array=("$@")


# Get the package path
PACKAGE_PATH=$(rospack find ${array[0]})

# Check if rospack succeeded
if [ -z "$PACKAGE_PATH" ]; then
    echo "Failed to find the ROS package '$0'."
    exit 1
fi

# Iterate over each argument passed to the script
for ((idx=1; idx<${#array[@]}; ++idx)); do
    # Check if the file exists
    if [ ! -f "$PACKAGE_PATH/config/${array[idx]}" ]; then
        echo "File does not exist: $PACKAGE_PATH/config/${array[idx]}"
        continue # Continue with the next file name instead of exiting
    fi

    # Load parameters contained in the file
    cnr_param_server --path-to-file "$PACKAGE_PATH/config/${array[idx]}"
done

# Exit gracefully by returning a status
exit 0
