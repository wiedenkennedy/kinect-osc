#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 420, 800, OF_WINDOW);
	ofRunApp(new testApp());
}
