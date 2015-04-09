## Declaration of variables

# eigen3 must be included with -I/usr/include/eigen3 (-l does not work, and
# #include <eigen3/Eigen/etc> is not recommended)

#compilers
NVCC = /opt/cuda/bin/nvcc
CC = g++

#includes
CUDA_HELPERS = -I/opt/cuda/sdk/common/inc
CUDA_INCLUDES = -I/opt/cuda/include/ -L/opt/cuda/lib64/
EIGEN_INCLUDES = -I/usr/include/eigen3/

INCLUDES = $(EIGEN_INCLUDES) $(CUDA_INCLUDES) $(CUDA_HELPERS)

#libraries
G2O_LIB = -L/home/gato/g2o/trunk/lib/ -lg2o_core -lg2o_stuff -lg2o_solver_csparse -lg2o_csparse_extension

LIBS = -lGL -lGLU -lcudart -lcuda -lboost_system -lglut -lGLEW -L/usr/lib64 -lopencv_core -lopencv_gpu -lopencv_highgui -lopencv_imgproc -lcxsparse $(G2O_LIB) -lCGAL -lgmp

#compiler arguments
CC_FLAGS = $(INCLUDES) -std=c++11 --verbose -g

#linker arguments
LD_FLAGS = $(INCLUDES) $(LIBS)

## File names
EXEC = pjgaetan

SRC_cpp = $(wildcard *.cpp)
SRC_cu = $(wildcard *.cu)

OBJ = $(SRC_cpp:.cpp=.o) $(SRC_cu:.cu=.o)

## Main target: assembler
$(EXEC): $(OBJ)
	$(CC) $(CC_FLAGS) $(LD_FLAGS) $(OBJ) -o $(EXEC)

## To obtain object files: compiler
# warning: no .cpp file must have the same name as a .cu file, or
# else Make will be confused since they compile to the same .o file
# example: (1) main.cpp main.cu   -> main.o main.o   //bad!  Make no understand!! :(
#          (2) main.cpp kernel.cu -> main.o kernel.o //good! Make happily makes!! :)

%.o: %.cpp
	$(CC) -c -march=native $(CC_FLAGS) $< -o $@

%.o: %.cu
	$(NVCC) -c -arch=sm_20 $(CC_FLAGS) $< -o $@

## To remove generated files
.PHONY: clean

clean:
	rm -f *.o $(EXEC)
