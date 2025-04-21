# ROS2_Package_Template
Template project for ROS2 with automated pipelines

## What to change when adopting this template
There are three things that need to be changed when adopting this template:
- Package.xml
- CMakeLists.txt
- Mkdocs.yml with documentation

### Package.xml
In the package.xml the following things needs to be changed:

- The package name:
```
  <name>custom_pkg</name>
```
- The package description:
```
  <description>
  Custom Pkg template
  </description>
```
- The maintainer:
```
  <maintainer email="example@example.com">example</maintainer>
```

### CMakeLists.txt
In the CMakeLists the following things need to be changed:

- The projectname needs to be your package name defined in package.xml:
```cmake
project(custom_pkg C CXX)
```

### Mkdocs.yml
In the Mkdocs.yml you need to change the site-name:
```
site_name: ROS2_Package_Template
```

### Docs
The docs files to change are the overview.md, implementation.md, ros2_pixi_build_linux_windows.md and usage_with_native_linux.md. 

## How to add the conda packages to the CLFML conda channel (CLFML members only!)?
See the instructions in the [CLFML conda channel repo](https://github.com/CLFML/conda_ros2_jazzy_channel).