# 分发专有的ROS软件包

## 1 为什么您需要专有的ROS软件包

专有的ROS（机器人操作系统）软件包在商业和敏感应用中提供了一系列的好处，尤其是在保护知识产权至关重要的情况下。通过分发专有的ROS软件包，您可以：

1. **保护知识产权**：确保您独特的算法、流程和创新保持机密。
2. **保持竞争优势**：防止竞争对手访问并潜在地复制您的解决方案。
3. **控制分发**：管理谁可以使用和修改您的软件，增强安全性和稳定性。
4. **创造收入**：将您的软件作为许可产品提供，通过许可协议创造潜在的收入来源。

## 2 专有 vs 开源ROS软件包

下表显示了开源ROS软件包和专有ROS软件包之间的区别：

| 特性           | 开源ROS软件包                                      | 专有ROS软件包                                      |
| -------------- | -------------------------------------------------- | -------------------------------------------------- |
| **可访问性**   | 源代码对任何人都是免费的，可以查看、修改和分发。   | 源代码对公众不可用，保护知识产权。                 |
| **协作**       | 鼓励社区贡献，促进快速改进和创新。                 | 贡献受限于内部团队或选择的合作伙伴。               |
| **透明度**     | 用户可以检查代码以查找安全漏洞或错误。             | 用户无法检查代码，依赖于提供商提供安全保证。       |
| **分发控制**   | 任何人都可以分发和使用软件包，通常在宽松的许可下。 | 分发和使用受控制，通常需要许可证。                 |
| **支持和维护** | 社区驱动的支持，质量和可用性可能不同。             | 通常由提供商提供官方支持和维护。                   |
| **成本**       | 通常免费使用和修改，尽管支持服务可能需要费用。     | 通常涉及许可费用，但包括专业支持。                 |
| **安全性**     | 开放式审查，可加速识别和修补漏洞。                 | 闭源减少了漏洞被暴露的风险，但依赖于内部审查流程。 |

## 3 制作和部署C++专有的ROS软件包

### 步骤1：准备您的ROS工作空间

确保您的ROS工作空间设置正确并且所有依赖关系都满足。进入您的工作空间：

```bash
cd ~/catkin_ws
```

### 步骤2：编译您的软件包

编译您的软件包以生成二进制文件：

```bash
catkin_make
```

### 步骤3：分发二进制文件

只分发编译后的二进制文件，而不是源代码。从`devel`或`install`目录复制二进制文件：

```bash
cp -r ~/catkin_ws/devel/lib/your_package /path/to/distribution/
```

**注意：**

- `/path/to/distribution/` 是您想要放置二进制文件的位置。通常，您可以将其移动到您的ROS软件包下的`bin`文件夹中。

**例如：**

- 如果您原始的ROS软件包结构如下所示：

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── src
  ```

- 那么您的专有ROS软件包结构将如下所示：

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

### 步骤4：创建必要的ROS文件

确保您的软件包包含`package.xml`和`CMakeLists.txt`以进行正确的ROS集成，即使您只分发二进制文件也是如此。

**示例 `package.xml`**

```xml
<package format="2">
  <name>your_package</name>
  <version>0.0.1</version>
  <description>您的专有ROS软件包</description>
  <maintainer email="your_email@example.com">您的姓名</maintainer>
  <license>专有</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <run_depend>roscpp</run_depend>
</package>
```

**示例 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(your_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()
```

**注意：**

- 如果在您原始的开源ROS软件包中，您的`CMakeLists.txt`包含以下代码行：

  ```cmake
  add_executable(my_node src/my_node.cpp)
  target_link_libraries(my_node $(catkin_LIBRARIES))
  ```

- 在专有ROS软件包的`CMakeLists.txt`中，您需要删除上述代码。

### 步骤5：部署软件包

在目标系统上，将二进制文件和必要的ROS文件放置在适当的目录中。确保环境设置正确：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# 将下面的内容更改为您自己的git

存储库。
git clone your_distributed_proprietary_ROS_Package.git
# 返回到catkin_ws
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

### 步骤6：运行节点

使用`rosrun`或`roslaunch`启动您的节点：

```bash
# 您可以尝试使用rosrun 
rosrun your_package your_binary
# 或者您可以使用roslaunch
roslaunch your_package your_launch_file.launch
```

## 4 制作和部署Python专有ROS软件包

### 4.1 bin方法

#### 步骤1：准备您的ROS工作空间

确保您的工作空间设置正确并且所有依赖关系都满足：

```bash
cd ~/catkin_ws/src/your_package/scripts
```

**注意：**

- `~/catkin_ws/src/your_package/scripts` 是您保存ROS Python源代码的路径。

#### 步骤2：使用PyInstaller打包您的脚本

安装PyInstaller：

```bash
pip install pyinstaller
```

打包您的Python脚本：

```bash
pyinstaller --onefile your_script.py
```

如何使用：

```
要生成什么：
  -D, --onedir          创建一个包含可执行文件的单文件夹包（默认）
  -F, --onefile         创建一个包含的单文件打包的可执行文件。
  --specpath DIR        生成的规范文件存储的文件夹（默认：当前目录）
  -n NAME, --name NAME  分配给捆绑应用程序和规范文件的名称（默认：第一个脚本的基本名称）
```

#### 步骤3：分发可执行文件

将生成的可执行文件从`dist`目录复制到您的分发位置：

```bash
cp dist/your_script /path/to/distribution/
```

**注意：**

- `/path/to/distribution/` 是您放置二进制文件的位置。通常，您可以将其放在ROS软件包的`bin`文件夹下。

**例如：**

- 如果您原始的ROS软件包结构如下所示：

  ```shell
  .
  ├── CMakeLists.txt
  ├── include
  ├── package.xml
  ├── launch
  └── src
  ```

- 那么您的专有ROS软件包结构将如下所示：

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

#### 步骤4：创建必要的ROS文件

确保您的软件包包含`package.xml`和`setup.py`以进行正确的ROS集成。

**示例 `package.xml`**：

```xml
<package format="2">
  <name>your_package</name>
  <version>0.0.1</version>
  <description>您的专有ROS软件包</description>
  <maintainer email="your_email@example.com">您的姓名</maintainer>
  <license>专有</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <run_depend>rospy</run_depend>
</package>
```

**示例 `setup.py`**

```python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['your_package'],
    package_dir={'': 'src'}
)

setup(**d)
```

#### 步骤5：部署软件包

在目标系统上，将可执行文件和必要的ROS文件放置在适当的目录中。确保环境设置正确：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

#### 步骤6：运行节点

使用`rosrun`或`roslaunch`启动您的节点：

```bash
# 您可以尝试使用rosrun 
rosrun your_package your_binary
# 或者您可以使用roslaunch
roslaunch your_package your_launch_file.launch
```

通过遵循这些步骤，您可以以专有方式分发您的ROS软件包，保护您的知识产权，同时利用ROS的强大和灵活性。

### 4.2 pyc 方法

- English version please check [here](https://github.com/zhz03/distribute_proprietary_ROS/blob/main/example_code/python_example/target_code/my_rostopics/distribute_pyc.md)
- 中文请点击[这里](https://github.com/zhz03/distribute_proprietary_ROS/blob/main/example_code/python_example/target_code/my_rostopics/pyc%E7%9A%84%E6%96%B9%E6%B3%95%E8%BF%9B%E8%A1%8C%E5%88%86%E5%8F%91.md)

