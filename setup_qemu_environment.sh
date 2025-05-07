#!/bin/bash
# Author: Hao Cheng

# Set the target architecture
ARCH="arm64"
TARGET_ARCH="linux/$ARCH"
WORKSPACE_DIR=$(pwd)

TAG="chenyuwuai/ros2_crosscompile:arm64-$(arch)-$(date +%Y%m%d)"

# Gain root access
if [ "$EUID" -ne 0 ]; then
    sudo echo -e "\033[1;32mGaining root access..."
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to gain root access"
        echo -e "\033[1;31mFailed to gain root access"
        exit 1
    fi
    echo -e "\033[1;33mRoot access granted\033[0m"
fi

# Check installed
if ! docker >/dev/null 2>&1; then
    echo -e "\033[1;31mDocker is not installed. Please install it first.\033[0m"
    exit 1
fi

# Pre Cross-compile
echo -e "\033[1;32mPre Cross-compile setup\033[0m"
echo -e "\033[1;32mChecking if qemu is installed\033[0m"
# Docker run docker run --privileged --rm tonistiigi/binfmt --install all
if ! docker run --privileged --rm tonistiigi/binfmt --install all >/dev/null 2>&1; then
    echo -e "\033[1;31mFailed to install qemu\033[0m"
    exit 1
fi
echo -e "\033[1;32mQemu installed successfully\033[0m"

# Prebuild OpenCV
echo -e "\033[1;32mPrebuild OpenCV\033[0m"
mkdir -p $WORKSPACE_DIR/build
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to create the build folder\033[0m"
    exit 1
fi
cd $WORKSPACE_DIR/build

# Try git clone https://github.com/opencv/opencv if opencv-xx folder does not exist
OPENCV_DIR=$(ls -d opencv-*)
# Check if the OpenCV directory exists
if [ ! -d "$OPENCV_DIR" ]; then
    echo -e "\033[1;34mCloning OpenCV repository...\033[0m"
    
    # Download OpenCV tarball using wget
    DOWNLOAD_ATTEMPTS=10
    SUCCESS=false

    for i in $(seq 1 $DOWNLOAD_ATTEMPTS); do
        echo -e "\033[1;33mAttempt $i/$DOWNLOAD_ATTEMPTS to download OpenCV tarball\033[0m"
        wget https://github.com/opencv/opencv/archive/refs/tags/4.10.0.tar.gz -O opencv-4.10.0.tar.gz

        if [ $? -eq 0 ]; then
            SUCCESS=true
            break
        fi
        # Wait a bit before retrying (optional)
        sleep 5
    done

    if [ "$SUCCESS" = false ]; then
        echo -e "\033[1;31mFailed to download OpenCV tarball after $DOWNLOAD_ATTEMPTS attempts\033[0m"
        exit 1
    fi

    # Extract the downloaded tarball
    echo -e "\033[1;34mExtracting OpenCV tarball...\033[0m"
    tar -xzf opencv-4.10.0.tar.gz

    # Cleanup the tarball
    rm opencv-4.10.0.tar.gz

    OPENCV_DIR=$(ls -d opencv-*)
else
    echo -e "\033[1;34mOpenCV repository already exists, skipping clone...\033[0m"
fi

# Verify that the OpenCV directory was created successfully
if [ ! -d "$OPENCV_DIR" ]; then
    echo -e "\033[1;31mOpenCV directory not found\033[0m"
    exit 1
fi

echo -e "\033[1;32mOpenCV prebuild setup completed successfully\033[0m"

echo -e "\033[1;32mBuilding the docker image for $ARCH\033[0m"

# Try build if $OPENCV_DIR/install does not exist
if [ ! -d "$OPENCV_DIR/install" ]; then
    echo -e "\033[1;34mBuilding OpenCV...\033[0m"
    cd $OPENCV_DIR
    mkdir build
    cd build
    cmake .. \
      -G Ninja -D CMAKE_TOOLCHAIN_FILE=../platforms/linux/aarch64-gnu.toolchain.cmake \
      -D CMAKE_INSTALL_PREFIX=../install \
      -D BUILD_opencv_objdetect=ON \
      -D BUILD_opencv_calib3d=OFF \
      -D BUILD_opencv_dnn=OFF \
      -D BUILD_opencv_features2d=OFF \
      -D BUILD_opencv_flann=OFF \
      -D BUILD_opencv_gapi=OFF \
      -D BUILD_opencv_highgui=OFF \
      -D BUILD_opencv_photo=OFF \
      -D BUILD_opencv_stitching=OFF \
      -D BUILD_opencv_ts=OFF \
      -D BUILD_opencv_ml=OFF \
      -D BUILD_opencv_video=OFF \
      -D WITH_OPENCL=OFF \
      -D WITH_OPENCL_SVM=OFF \
      -D WITH_OPENMP=OFF \
      -D WITH_JPEG=ON \
      -D WITH_PNG=ON \
      -D WITH_TIFF=OFF \
      -D WITH_WEBP=OFF \
      -D WITH_JASPER=OFF \
      -D WITH_OPENEXR=OFF \
      -D BUILD_TESTS=OFF \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D OPENCV_ENABLE_NONFREE=OFF
    

    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to configure OpenCV\033[0m"
        exit 1
    fi

    # Build OpenCV
    ninja -j$(nproc)

    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to build OpenCV\033[0m"
        exit 1
    fi

    ninja install
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to install OpenCV\033[0m"
        exit 1
    fi
fi

# Environment Variables for OPENCV_INSTALL_DIR
export OPENCV_INSTALL_DIR=./build/$OPENCV_DIR/install
echo -e "\033[1;32mOPENCV_INSTALL_DIR: $OPENCV_INSTALL_DIR\033[0m"
cd $WORKSPACE_DIR

# Build the docker image, if fails, stop the script
docker build . --platform=$TARGET_ARCH -t $TAG --load --network host \
    --build-arg OPENCV_INSTALL_DIR=$OPENCV_INSTALL_DIR

if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to build the docker image\033[0m"
    exit 1
fi

docker run $TAG "echo 'Build successful'" >/dev/null 2>&1
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to run the container, trying to reinitialize the qemu environment\033[0m"
    docker run --privileged --rm tonistiigi/binfmt --install all
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to reinitialize the qemu environment\033[0m"
        exit 1
    fi
    docker run $TAG "echo 'Build successful'" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to run the container\033[0m"
        exit 1
    fi
fi

echo -e "\033[1;32mBuild complete. Copying the install folder to the workspace\033[0m"

# Need to run the container to copy the install folder
CONTAINER_ID=$(docker container ls -a | grep $TAG | awk 'NR==1 {print $1}')
if [ -z "$CONTAINER_ID" ]; then
    echo -e "\033[1;31mFailed to get the container ID\033[0m"
    exit 1
fi

# Copy the install folder to the workspace, remove first if it exists
if [ -d "$WORKSPACE_DIR/ubuntu_$ARCH" ]; then
    sudo rm -rf $WORKSPACE_DIR/ubuntu_$ARCH
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to remove the existing install folder\033[0m"
        exit 1
    fi
fi

echo -e "sudo docker cp $CONTAINER_ID:/ $WORKSPACE_DIR/ubuntu_$ARCH"
sudo docker cp $CONTAINER_ID:/ $WORKSPACE_DIR/ubuntu_$ARCH
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to copy the install folder\033[0m"
    exit 1
fi
echo -e "\033[1;32mInstall folder copied to the workspace\033[0m"

# Add COLCON_IGNORE file to the install folder
echo -e "\033[1;32mAdding COLCON_IGNORE file to the install folder\033[0m"
sudo touch $WORKSPACE_DIR/ubuntu_$ARCH/COLCON_IGNORE
if [ $? -ne 0 ]; then
    echo -e "\033[1;31mFailed to add COLCON_IGNORE file\033[0m"
    exit 1
fi

# Mkdir for workspace
if [ ! -d "$WORKSPACE_DIR/ubuntu_$ARCH/workspace" ]; then
    sudo mkdir $WORKSPACE_DIR/ubuntu_$ARCH/workspace
    sudo chown $USER:$USER $WORKSPACE_DIR/ubuntu_$ARCH/workspace
    if [ $? -ne 0 ]; then
        echo -e "\033[1;31mFailed to create the workspace folder\033[0m"
        exit 1
    fi
fi

# Install the necessary packages
echo -e "\033[1;32mInstalling the necessary packages for cross-compilation\033[0m"
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu -y

echo -e "\033[1;32mSetup complete\033[0m"
