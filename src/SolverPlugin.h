#pragma once

#ifndef SOLVERPLUGIN_H
#define SOLVERPLUGIN_H

#include "Newton.h"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/ViewerPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <thread>

#ifndef INT_INF
#define INT_INF numeric_limits<int>::max()
#endif

using namespace std;
using namespace igl::opengl::glfw;

class SolverPlugin : public ViewerPlugin
{
public:
	SolverPlugin();
	void init(Viewer *viewer);
	void post_resize(int w, int h); // external resize due to user interaction

	bool load(string filename);
	void export_uv_to_obj();
// 	void add_texture_slider(nanogui::Window* window, double& var, const string& name);
	void initialize();
	
	bool mouse_move(int mouse_x, int mouse_y);
	void MoveHandle(int mouse_x, int mouse_y);
	bool process_mouse_move();

	bool mouse_down(int button, int modifier);

	void RemoveHandle(int index);

	void AddHandle(int &vid);
	bool mouse_up(int button, int modifier);
    void shutdown();
    bool mouse_scroll(float delta_y);
	
	bool pre_draw();
	bool key_down(int key, int modifiers);
	bool key_up(int key, int modifiers);

	void update_mesh();
	void start_solver_thread();
	void stop_solver_thread();

	bool rotation_enabled = false;
	bool translation_enabled = false;
	bool uv_translation_enabled = false;

	int last_mouse_x, last_mouse_y;

	thread solver_thread;
	unique_ptr<Newton> solver;
	shared_ptr<TotalObjective> totalObjective;

	// lambda slider
// 	Slider *slider;

	unsigned int processed_mesh_id = 0, source_mesh_id = 0;

	float texture_size = 0.5;
	double max_texture_val = 10.;

	int solver_iter_counter = 0;
	bool use_num_steps = false;

	int resolution_factor = 4;

	bool update_colors = false;

	shared_ptr<double> dist_color_clamp = make_shared<double>(1.0);
	shared_ptr<double> max_dist_color_value = make_shared<double>(4.0);

	string mesh_filename;

private:
	// Pointer to the imgui
	igl::opengl::glfw::imgui::ImGuiMenu menu;

	// The 3d mesh
	Eigen::MatrixX3d V;
	MatX3i F;

	// Rotation matrices
	Eigen::MatrixXd Rx, Ry;

	Eigen::MatrixX3d uv_triangle_colors, mesh_triangle_colors;
	int hovered_triangle = -1, hovered_vertex = -1;
	
	Eigen::MatrixX3d mesh_pos_down, uv_mesh_pos_down;
	RVec3 last_mouse_pos, projected_mouse_down, point_pos_down;
	RVec3 point_pos;
	bool move_point = false;
	Eigen::MatrixX3d mesh_3d_normals_down;

	bool mesh_loaded = false;
	bool mouse_on_uv_side = false;
	bool mouse_over_3d_mesh = false;
	bool mouse_over_2d_mesh = false;
	bool mouse_over_uv_mesh = false;
	bool release_when_translation_done = false;

	// coords used in process_mouse_move
	int curr_mouse_x, curr_mouse_y;

	// some mouse states
	bool MOUSE_LEFT = false;
	bool MOUSE_MID = false;
	bool MOUSE_RIGHT = false;

	bool show_distortion_error = false;
	bool colorByRGB=false;
	Eigen::MatrixX3d RGBColors;

	//colors
	RVec3 C, C_hover, white, red, C_merge, zero3, ones3, black;

	int rightView;
	int leftView;
	int FindHitVertex();
	int FindHitHandle();
	int selectedHandle=-1;
	std::vector<int>& HandlesInd; //reference to indices in constraitPositional
	MatrixX2d& HandlesPos; //reference to positions in constraitPositional
};

#endif