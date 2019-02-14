# NON-PLANAR 3D PRINTING TOOLPATH GENERATION

## LIBRARIES REQUIRED:
1. Eigen (http://eigen.tuxfamily.org/index.php?title=Main_Page)
2. Libigl (https://github.com/libigl/libigl)

## INSTALLATION INSTRUCTIONS FOR LIBIGL AND DEPENDANCIES:

1. download zip or clone git directory (https://github.com/libigl/libigl)
2. install all the dependancies listed
```
	sudo apt-get install git
	sudo apt-get install build-essential
	sudo apt-get install cmake
	sudo apt-get install libx11-dev
	sudo apt-get install mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev
	sudo apt-get install libxrandr-dev
	sudo apt-get install libxi-dev
	sudo apt-get install libxmu-dev
	sudo apt-get install libblas-dev
	sudo apt-get install libxinerama-dev
	sudo apt-get install libxcursor-dev
```
2. build steps :-
```
			mkdir build
			cd build/
			cmake ..
			make -j
			sudo make install
```

3. NOTE : If only header files are added in the /usr/local/include/igl folder, then add all the cpp and extra support files from cloned libigl source directory (copy all files from ${SOURCE_DIR}/include/igl/ )

4. To avoid issue of 'pythread', add [target_link_libraries(${EXECUTABLE_FILE} pthread) to cmake]

## RUNNING MAIN FILE:

1. add stl file in the data/
2. specify appropriate parameters in main_file.cpp
3. to generate the .out file-
   in the main directory-
   ```
   catkin_make
   roscore
   rosrun tool_path_planning generate_path_server
   tool_path_planning generate_path_client /home/user/data/test_part2.STL
	```
