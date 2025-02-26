# Arduino-Based Obstacle Course Robot

## Project Description  

This project is a first-semester university assignment at Aberystwyth University for the **Artificial Intelligence and Robotics** course.  
The objective was to program an **Arduino-based robot** in **Arduino C** to navigate an obstacle course.  

The robot was designed to:  

- **Follow a black strip** using light-dependent resistors (LDRs).  
- **Avoid obstacles** and rejoin the strip.  
- **Scan barcodes** to interpret navigation instructions.  
- **Adapt its path** based on barcode instructions and follow different line colors.  

This project served as an introduction to **embedded systems programming, sensor integration, and motor control**.  

## File Structure  

- `Robotics_Assignment.ino` – Main Arduino sketch containing the robot’s control logic.  
- `Robotics_Assignment_Report.pdf` – A detailed report outlining the project, challenges faced, and results.  

## Prerequisites  

- **Arduino IDE** (latest version recommended).  
- **Arduino Uno or compatible microcontroller**.  
- **IR sensors, LDRs, and servos** (configured for obstacle detection and movement).  

## Setup and Installation  

1. **Install the Arduino IDE** from the [official website](https://www.arduino.cc/en/software).  
2. **Connect the Arduino board** via USB.  
3. **Open the `Robotics_Assignment.ino` file** in the Arduino IDE.  
4. **Compile and upload the code** to the Arduino board.  

## Robot Functionality  

### 1. Line Following  

- Uses **LDR sensors** to detect surface colors (black strip vs. background).  
- Calibrates LDR values for precise detection.  
- Adjusts servo speeds to maintain course using **proportional control**.  

### 2. Obstacle Avoidance  

- Uses an **IR transmitter and receiver** to detect obstacles.  
- Implements a **right-turn, forward-move, left-turn strategy** to bypass obstacles.  
- Rejoins the original path after avoidance maneuvers.  

### 3. Barcode Scanning  

- Reads **barcode-like patterns** with LDRs.  
- Uses `millis()` to measure bar widths and gaps.  
- Compares patterns to predefined commands.  

### 4. Finishing the Course  

- Based on barcode data, the robot:  
  - **Continues forward** if the strip is the same.  
  - **Changes direction** if a different barcode is read.  
  - **Turns around** if instructed.  

## Troubleshooting  

- **Robot not moving straight** → Adjust servo stop values in EEPROM.  
- **Obstacle avoidance failing** → Ensure IR sensors are properly wired and functional.  
- **Barcode scanning not working** → Check LDR calibration and timing accuracy.  

## Results  

✅ **Line following:** Successfully implemented.  
🔶 **Obstacle avoidance:** Partially successful.  
❌ **Barcode scanning & finish detection:** Not fully completed due to timing issues.  

## License  

This project is shared for educational purposes. Feel free to modify and improve!  
