#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include "SolverPlugin.h"
#include "TotalObjective.h"

using namespace std;

int main(int argc, char** argv)
{
	//unsigned int num_threads = max(atoi(getenv("OMP_NUM_THREADS")), 1);
	//omp_set_num_threads(num_threads);

	Viewer viewer;

	SolverPlugin solverPlugin;
	viewer.plugins.push_back(&solverPlugin);

	// start viewer
	viewer.launch(true, false,1920,1080);
	return 0;
}