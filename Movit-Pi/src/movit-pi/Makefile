CPP_FILES = $(wildcard *.cpp)
OBJ_FILES = $(addprefix $(OBJ_DIR_MOVIT_PI)/,$(CPP_FILES:.cpp=.o))

$(TARGET_MOVIT_PI): $(OBJ_FILES)
	$(CC) $(CXXFLAGS) bcm2835.c -o $(OBJ_DIR_MOVIT_PI)/bcm2835.o


$(OBJ_DIR_MOVIT_PI)/%.o: %.cpp
	$(CPP) $(CXXFLAGS) $(CPPFLAGS) $< -o $@
