cd ../3rd
./conan_install.sh  # option for osqp
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Debug -DCATKIN_WHITELIST_PACKAGES=""