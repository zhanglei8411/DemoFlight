TARGET = ../output/DJI_Onboard_API_Cmdline_Test

OBJECTS = main.o DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Sample.o DJI_Pro_Codec.o DJI_Pro_Rmu.o tinyxml2.o XY_LL.o XY_Route.o XY_Ctrl.o XY_Debug.o

CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -Isrc/ -I/src/DJI_LIB -I/src/XY_APP

LDFLAGS = -lpthread

CXX = g++

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
main.o : ../src/main.cpp
	$(CXX) $(CFLAGS) -c ../src/main.cpp

DJI_Pro_App.o : ../src/DJI_LIB/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../src/DJI_LIB/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../src/DJI_LIB/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Link.cpp

DJI_Pro_Sample.o : ../src/DJI_Pro_Sample.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Sample.cpp
	
DJI_Pro_Codec.o : ../src/DJI_LIB/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Codec.cpp

DJI_Pro_Rmu.o: ../src/DJI_LIB/DJI_Pro_Rmu.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Rmu.cpp

tinyxml2.o : ../src/tinyxml2.cpp
	$(CXX) $(CFLAGS) -c ../src/tinyxml2.cpp

XY_LL.o : ../src/XY_APP/XY_LL.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_APP/XY_LL.cpp

XY_Route.o : ../src/XY_APP/XY_Route.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_APP/XY_Route.cpp

XY_Ctrl.o : ../src/XY_APP/XY_Ctrl.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_APP/XY_Ctrl.cpp
	
XY_Debug.o : ../src/XY_APP/XY_Debug.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_APP/XY_Debug.cpp

clean:
	-rm *.o
	-rm $(TARGET)
