APP_NAME = Maze
BUILD_DIR = ./bin
C_FILES = ./src/*.cpp

APP_DEFINES:=
APP_INCLUDES:= -I./src/vendors/GLFW -framework Cocoa -framework OpenGL -framework IOKit
APP_LINKERS:= -L./src/vendors/GLFW/lib -lglfw3

build: 
	clang++ $(C_FILES) -o $(BUILD_DIR)/$(APP_NAME) $(APP_INCLUDES) $(APP_LINKERS)