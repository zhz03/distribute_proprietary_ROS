# Distribute Proprietary ROS Package

- This is EN version
- 你也可以选择[中文]()

## 1 Why You Need Proprietary ROS Packages

Proprietary ROS (Robot Operating System) packages offer a range of benefits, particularly in commercial and sensitive applications where protecting intellectual property is crucial. By distributing proprietary ROS packages, you can:

1. **Protect Intellectual Property**: Ensure your unique algorithms, processes, and innovations remain confidential.
2. **Maintain Competitive Edge**: Prevent competitors from accessing and potentially replicating your solutions.
3. **Control Distribution**: Regulate who can use and modify your software, enhancing security and stability.
4. **Generate Revenue**: Offer your software as a licensed product, creating potential revenue streams through licensing agreements.

## 2  Proprietary VS. Open Source ROS Packages

The difference between open source ROS package and proprietary ROS package is shown in the table below: 

| Feature                     | Open Source ROS Packages                                     | Proprietary ROS Packages                                     |
| --------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| **Accessibility**           | Source code is freely available for anyone to view, modify, and distribute. | Source code is not available to the public, protecting intellectual property. |
| **Collaboration**           | Encourages community contributions, leading to rapid improvements and innovation. | Contributions are restricted to internal teams or selected partners. |
| **Transparency**            | Users can inspect the code for security vulnerabilities or bugs. | Users cannot inspect the code, relying on the provider for security assurances. |
| **Distribution Control**    | Anyone can distribute and use the package, often under permissive licenses. | Distribution and use are controlled, typically requiring a license. |
| **Support and Maintenance** | Community-driven support, which may vary in quality and availability. | Often accompanied by official support and maintenance from the provider. |
| **Cost**                    | Generally free to use and modify, though support services may cost money. | Often involves licensing fees, but includes professional support. |
| **Security**                | Open to scrutiny, which can lead to faster identification and patching of vulnerabilities. | Closed source reduces the risk of vulnerabilities being exposed, but relies on internal review processes. |

## 3 Make and Deploy a C++ Proprietary ROS Package 

### Step 1: Prepare Your ROS Workspace
Ensure your ROS workspace is properly set up and all dependencies are met. Navigate to your workspace:
```bash
cd ~/catkin_ws 
```

### Step 2: Compile Your Package
Compile your package to generate the binary files:
```bash
catkin_make 
```

### Step 3: Distribute Binary Files 
Only distribute the compiled binary files, not the source code. Copy the binaries from the `devel` or `install` directory: 
```bash
cp -r ~/catkin_ws/devel/lib/your_package /path/to/distribution/
```

**Note:** 

- `/path/to/distribution/` is where you want to put your binary file. Normally, you can move this to your ros package under `bin` folder. 

**For Example:**

- If your original ros package structure like this:

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── src
  ```

- Then your proprietary ros package structure will be like this:

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── bin
  		├── your_bin_file
  		└── your_bin_file2
  ```

### Step 4: Create the Necessary ROS Files

Ensure your package includes `package.xml` and `CMakeLists.txt` for proper ROS integration, even if you are distributing only binaries.

### Example `package.xml`
```xml
<package format="2">
  <name>your_package</name>
  <version>0.0.1</version>
  <description>Your proprietary ROS package</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <run_depend>roscpp</run_depend>
</package>
```

### Example `CMakeLists.txt`
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(your_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()
```

**Note**: 

- If in your original open-source ROS package, your `CMakeLists.txt` contains following lines of code

  ```cmake
  add_executable(my_node src/my_node.cpp)
  target_link_libraries(my_node $(catkin_LIBRARIES))
  ```

- In the ``CMakeLists.txt`  of Proprietary ROS package, you need to delete the above code. 

### Step 5: Deploy the Package

On the target system, place the binary files and necessary ROS files in the appropriate directories. Ensure the environment is set up correctly:
```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# change the following into your own git repo.
git clone your_distributed_proprietary_ROS_Package.git 
# navigate back to catkin_ws
cd .. 
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### Step 6: Run Your Node
Use `rosrun` or `roslaunch` to start your node:
```bash
# you can try rosrun 
rosrun your_package your_binary
# or you can use roslaunch 
roslaunch your_package your_launch_file.launch
```

## 4 Make and Deploy Python ROS  Proprietary Package

### Step 1: Prepare Your ROS Workspace
Ensure your workspace is set up and all dependencies are met:
```bash
cd ~/catkin_ws/src/your_package/scripts
```

Note:

- `~/catkin_ws/src/your_package/scripts` is the path where you save your ros python source code.

### Step 2: Use PyInstaller to Package Your Script

Install PyInstaller:
```bash
pip install pyinstaller
```

Package your Python script:
```bash
pyinstaller --onefile your_script.py
```

How to use: 

```
What to generate:
  -D, --onedir          Create a one-folder bundle containing an executable (default)
  -F, --onefile         Create a one-file bundled executable.
  --specpath DIR        Folder to store the generated spec file (default: current directory)
  -n NAME, --name NAME  Name to assign to the bundled app and spec file (default: first script's basename)
```

### Step 3: Distribute the Executable

Copy the generated executable from the `dist` directory to your distribution location:
```bash
cp dist/your_script /path/to/distribution/
```

Note:

- `/path/to/distribution/` is where you put your bin file, normally, you can put it in your ros package under `bin` folder

**For Example:**

- If your original ros package structure like this:

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── src
  ```

- Then your proprietary ros package structure will be like this:

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── bin
  		├── your_bin_file
  		└── your_bin_file2
  ```

### Step 4: Create the Necessary ROS Files

Ensure your package includes `package.xml` and `setup.py` for proper ROS integration.

### Example `package.xml`
```xml
<package format="2">
  <name>your_package</name>
  <version>0.0.1</version>
  <description>Your proprietary ROS package</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <run_depend>rospy</run_depend>
</package>
```

### Example `setup.py`
```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['your_package'],
    package_dir={'': 'src'}
)

setup(**d)
```

### Step 5: Deploy the Package
On the target system, place the executable and necessary ROS files in the appropriate directories. Ensure the environment is set up correctly:
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Step 6: Run Your Node
Use `rosrun` or `roslaunch` to start your node:
```bash
# you can try rosrun 
rosrun your_package your_binary
# or you can use roslaunch 
roslaunch your_package your_launch_file.launch
```

By following these steps, you can distribute your ROS packages in a proprietary manner, protecting your intellectual property while still leveraging the power and flexibility of ROS.