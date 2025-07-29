## Unicore UM982 ROS 2 Driver: Implementation Plan

This document provides a detailed breakdown of the tasks required to build, test, and deploy the Unicore UM982 ROS 2 Driver. The project will be developed in C++ using the `rclcpp` library and the `ament_cmake` build system, with Git for version control.

### Phase 0: Project Setup and Version Control

**Goal:** Establish the project's foundation with a version-controlled repository.

*   #### **Task 1: Initialize Git Repository**
    *   **Dependencies:** None
    *   **Context:** This task creates the version control structure for the project, ensuring all subsequent work is tracked.
    *   **Subtasks:**
        *   [x] Create a new remote repository (e.g., on GitHub or GitLab) named `unicore_um982_driver`.
        *   [x] Clone the empty repository to your local development machine.
        *   [x] Create a `README.md` file with an initial project title.
        *   [x] Create a `.gitignore` file tailored for ROS 2 and C++ projects. This should include common build artifacts like `build/`, `install/`, `log/`, and editor-specific files.
        *   [x] Add and commit the `README.md` and `.gitignore` files to the `main` branch.
        *   [x] Push the initial commit to the remote repository.
    *   **Intermediate Test:**
        *   [x] The remote repository should contain the `README.md` and `.gitignore` files.
        *   [x] Running `git status` in the local repository should report that the working tree is clean.

### Phase 1: Core Node Setup and Compilation

**Goal:** Establish a basic ROS 2 package structure that can be compiled and executed.

*   #### **Task 2: Create the ROS 2 Package**
    *   **Dependencies:** Task 1
    *   **Context:** This task creates the fundamental directory structure and build files for the driver node within the version-controlled workspace.
    *   **Subtasks:**
        *   [x] From the `main` branch, create and switch to a new feature branch: `git checkout -b feature/core-node`.
        *   [x] Use `ros2 pkg create` to generate a new package named `unicore_um982_driver` with a build type of `ament_cmake` and a dependency on `rclcpp`.
        *   [x] Populate the `package.xml` with the author, license, and description.
        *   [x] Create a `src/unicore_um982_driver_node.cpp` file with a minimal `rclcpp::Node` class and a `main` function.
        *   [x] Add the executable to `CMakeLists.txt` and link it against `rclcpp`.
    *   **Intermediate Test:**
        *   [x] Run `colcon build`. The build must complete without errors.
        *   [x] Source `install/setup.bash` and run `ros2 run unicore_um982_driver unicore_um982_driver_node`. The node should start and run without crashing.
    *   **Version Control:**
        *   [x] Commit all the new package files to the `feature/core-node` branch with a clear message (e.g., "feat: Initial ROS 2 package structure").
        *   [x] Merge the `feature/core-node` branch back into the `main` branch.

### Phase 2: Serial Communication and Data Ingestion

**Goal:** Establish a connection to the UM982 receiver and read the raw data stream.

*   #### **Task 3: Implement Serial Port Communication**
    *   **Dependencies:** Task 2
    *   **Context:** This task focuses on integrating a C++ serial library to read data from the physical hardware.
    *   **Subtasks:**
        *   [x] Create and switch to a new feature branch: `git checkout -b feature/serial-comms`.
        *   [x] Add a suitable C++ serial library (e.g., `serial`) as a dependency.
        *   [x] In the `UnicoreDriverNode` class, add logic to open and configure the serial port with hardcoded values for initial testing (e.g., `/dev/ttyUSB0`, `230400`). Include robust error handling.
        *   [x] Create an `rclcpp::Timer` to periodically call a `readSerialData` method.
        *   [x] Implement the `readSerialData` method to read lines from the serial port and log them using `RCLCPP_INFO`.
    *   **Intermediate Test:**
        *   [x] Connect the UM982. Build and run the node.
        *   [x] Verify that raw `PVTSLN` messages are continuously printed to the terminal.
    *   **Version Control:**
        *   [x] Commit the changes to the `feature/serial-comms` branch.
        *   [ ] Merge the `feature/serial-comms` branch back into the `main` branch.

### Phase 3: Data Parsing and Structuring

**Goal:** Convert the raw string data from the receiver into a structured C++ object.

*   #### **Task 4: Develop the PVTSLN Parser**
    *   **Dependencies:** Task 3
    *   **Context:** This task involves writing the core logic to interpret the `PVTSLN` message format.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/parser`.
        *   [ ] Define a `PVTSLNData` C++ struct in a new header file to hold the parsed data.
        *   [ ] Create and implement a parser function: `bool parsePVTSLN(const std::string& line, PVTSLNData& data)`.
        *   [ ] Integrate the parser into the main `readSerialData` loop and log the structured data to verify correctness.
    *   **Intermediate Test:**
        *   [ ] Build and run the node.
        *   [ ] Verify that the console output shows structured, parsed data (e.g., "Parsed Heading: 123.45") and that the values appear correct.
    *   **Version Control:**
        *   [ ] Commit the parser implementation to the `feature/parser` branch.
        *   [ ] Merge the `feature/parser` branch back into the `main` branch.

### Phase 4: ROS Topic Publishing

**Goal:** Publish the parsed GPS and IMU data to the ROS 2 ecosystem.

*   #### **Task 5: Implement ROS 2 Publishers**
    *   **Dependencies:** Task 4
    *   **Context:** This task makes the driver useful by broadcasting data for other nodes to consume.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/publishing`.
        *   [ ] Add `sensor_msgs` and `geometry_msgs` as dependencies.
        *   [ ] Declare publishers for `sensor_msgs::msg::NavSatFix` (`gps/fix`) and `sensor_msgs::msg::Imu` (`gps/imu`).
        *   [ ] Implement methods to populate and publish these messages from the parsed `PVTSLNData` struct. This includes the heading-to-quaternion conversion.
    *   **Intermediate Test:**
        *   [ ] Build and run the node.
        *   [ ] Use `ros2 topic echo` on `/gps/fix` and `/gps/imu` to verify the messages are published correctly and at the expected rate.
    *   **Version Control:**
        *   [ ] Commit the publisher implementation to the `feature/publishing` branch.
        *   [ ] Merge the `feature/publishing` branch back into the `main` branch.

### Phase 5: Automatic Configuration and Parameterization

**Goal:** Make the driver robust and easy to use by automating setup and exposing settings.

*   #### **Task 6: Implement Automatic Hardware Configuration**
    *   **Dependencies:** Task 3
    *   **Context:** This task adds the logic to send configuration commands to the UM982 at startup.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/auto-config`.
        *   [ ] Create and implement a `configureReceiver()` method that writes the `LOG ... ONTIME 0.05` and `SAVECONFIG` commands to the serial port.
        *   [ ] Call this method in the node's constructor after the serial port is opened.
    *   **Intermediate Test:**
        *   [ ] Power cycle the GPS, confirm it's not streaming `PVTSLN`. Run the driver once. Power cycle again and confirm with a serial terminal that the GPS now streams `PVTSLN` automatically.
    *   **Version Control:**
        *   [ ] Commit the feature to the `feature/auto-config` branch.
        *   [ ] Merge the `feature/auto-config` branch back into the `main` branch.

*   #### **Task 7: Implement ROS 2 Parameterization**
    *   **Dependencies:** Task 6
    *   **Context:** This task replaces hardcoded values with configurable ROS 2 parameters.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/parameterization`.
        *   [ ] Use `declare_parameter` to define `port` and `baudrate` parameters.
        *   [ ] Replace hardcoded values with values retrieved from these parameters.
        *   [ ] Create a `config/unicore_driver_params.yaml` file with default values.
    *   **Intermediate Test:**
        *   [ ] Create a temporary launch file to load the YAML. Test that the node fails with an invalid port and runs with a valid one.
    *   **Version Control:**
        *   [ ] Commit the parameterization work to the `feature/parameterization` branch.
        *   [ ] Merge the `feature/parameterization` branch back into the `main` branch.

### Phase 6: System Integration and RTK Handling

**Goal:** Create a launch file to orchestrate the driver and the external NTRIP client.

*   #### **Task 8: Create the ROS 2 Launch File**
    *   **Dependencies:** Task 7
    *   **Context:** This task provides a single entry point for running the entire system.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/launch-file`.
        *   [ ] Create a `launch/unicore.launch.py` file.
        *   [ ] Add a `Node` action to start the driver, loading the YAML file.
        *   [ ] Add an `ExecuteProcess` action to run `str2str` with the NTRIP URL and `respawn=True`.
        *   [ ] Install the `launch` and `config` directories in `CMakeLists.txt`.
    *   **Intermediate Test:**
        *   [ ] Run `ros2 launch unicore_um982_driver unicore.launch.py`.
        *   [ ] Verify both processes are running and that the GPS fix status eventually indicates an RTK fix.
    *   **Version Control:**
        *   [ ] Commit the launch file and associated changes to the `feature/launch-file` branch.
        *   [ ] Merge the `feature/launch-file` branch back into the `main` branch.

### Phase 7: Diagnostics and Finalization

**Goal:** Implement health monitoring and prepare the package for distribution.

*   #### **Task 9: Implement Diagnostics Publisher**
    *   **Dependencies:** Task 5
    *   **Context:** This task provides standardized health reporting for system monitoring tools.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/diagnostics`.
        *   [ ] Add the `diagnostic_updater` dependency.
        *   [ ] Implement a `diagnostic_updater::Updater` to publish the fix status (OK, WARN, ERROR) and satellite count.
    *   **Intermediate Test:**
        *   [ ] Run the launch file and `rqt_runtime_monitor`.
        *   [ ] Verify the "RTK GPS" component appears with the correct status and updates properly.
    *   **Version Control:**
        *   [ ] Commit the diagnostics implementation to the `feature/diagnostics` branch.
        *   [ ] Merge the `feature/diagnostics` branch back into the `main` branch.

*   #### **Task 10: Final Documentation and Code Cleanup**
    *   **Dependencies:** Tasks 1-9
    *   **Context:** This final task involves polishing the code and writing a comprehensive `README.md`.
    *   **Subtasks:**
        *   [ ] Create and switch to a new feature branch: `git checkout -b feature/documentation`.
        *   [ ] Review all C++ code for clarity, comments, and style.
        *   [ ] Update the `README.md` file with full installation, configuration, and usage instructions.
    *   **Intermediate Test:**
        *   [ ] Have a colleague follow the `README.md` on a fresh machine to install, configure, and run the driver successfully.
    *   **Version Control:**
        *   [ ] Commit the final documentation and code cleanup to the `feature/documentation` branch.
        *   [ ] Merge the `feature/documentation` branch back into the `main` branch.
        *   [ ] Create a final version tag on the `main` branch: `git tag -a v1.0.0 -m "Initial stable release"`.
        *   [ ] Push the tag to the remote repository: `git push --tags`.
