#include "stdafx.h"
#include <windows.h>
#include <stdio.h>
#include <iostream>
#include "scene_io.h"
#include "Timer.h"
#include "Render.h"

//#define IMAGE_WIDTH	  800
//#define IMAGE_HEIGHT  600

#define IMAGE_WIDTH	500
#define IMAGE_HEIGHT  500

SceneIO *scene = NULL;


static void loadScene(char *name) {
	/* load the scene into the SceneIO data structure using given parsing code */
	scene = readScene(name);

	/* hint: use the Visual Studio debugger ("watch" feature) to probe the
	scene data structure and learn more about it for each of the given scenes */


	/* write any code to transfer from the scene data structure to your own here */
	/* */

	return;
}



int main(int argc, char *argv[]) {
	Timer total_timer;
	total_timer.startTimer();

	loadScene("../MyScenes/scene2.ascii");
	//loadScene("../Scenes/test3.scene");

	render(scene, IMAGE_WIDTH, IMAGE_HEIGHT, "../Renders/scene2.png");

	if (scene != NULL) {
		deleteScene(scene);
	}

	total_timer.stopTimer();
	fprintf(stderr, "Total time: %.5lf secs\n\n", total_timer.getTime());
	std::cout << std::endl;
	return 0;
}
