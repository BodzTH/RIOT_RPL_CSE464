#!/bin/bash

# 1. Cleanup: Kill any previously running instances to start fresh
echo "Cleaning up old processes..."
pkill -f gnrc_networking.elf

# 2. Loop to start 10 nodes (from tap0 to tap9)
for i in {0..9}
do
    echo "Starting node on tap$i..."
    
    # Start the RIOT binary directly using the ELF file
    # We use '&' to run it in the background
    ./bin/native64/gnrc_networking.elf tap$i &
    
    # Add a small delay (0.5s) to prevent race conditions during startup
    sleep 0.5
done

echo "------------------------------------------------"
echo "Success! All 10 nodes are running in background."
echo "Use 'ps aux | grep gnrc_networking' to verify."
echo "------------------------------------------------"
# Loop from 0 to 9 to start 10 nodes
for i in {0..9}
do
    echo "Starting node on tap$i..."
    
    # Start the RIOT instance using the specific tap interface in the background
    make BOARD=native PORT=tap$i term &
    
    # Small delay to ensure the node starts up cleanly before the next one
    sleep 1
done

echo "All 10 nodes are up and running!"

