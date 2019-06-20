cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

project( main )
find_package( OpenCV REQUIRED )
find_package(Qt4 4.4.3 REQUIRED QtGui QtXml)

add_executable( main main.cpp detection.cpp )
target_link_libraries( main ${OpenCV_LIBS}  Qt4::QtGui Qt4::QtXml )
