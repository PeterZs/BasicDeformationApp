#include "SolverPlugin.h"

#include "svg_exporter.h"

#include <queue>
#include <deque>
#include <array>
#include <unordered_set>
#include <fstream>
#include <algorithm>
#include <igl/writeOBJ.h>
#include <igl/unproject.h>
#include <igl/unproject_ray.h>
#include <igl/file_dialog_save.h>
#include <igl/opengl/ViewerData.h>
#include <igl/unproject_onto_mesh.h>

SolverPlugin::SolverPlugin()
	: solver(make_unique<Newton>()), totalObjective(make_shared<TotalObjective>())

{
	C << 0.0, 1.0, 0.0;
	C_hover << 0.854902, 0.647059, 0.12549;
	white << 0.7, 0.7, .4;
	red << 1.0, 0.0, 0.0;
	C_merge << 51. / 255., 204. / 255., 1.0;
	zero3 << 0., 0., 0.;
	ones3 << 1., 1., 1.;
	black << 0., 0., 0.;
}

void SolverPlugin::init(Viewer *viewer)
{
	leftView = 0;
	viewer->core->background_color << 1., 1., 1., 1.; // 0.25, 0.25, 0.25, 1.0;
	viewer->core->orthographic = true;
	viewer->core->viewport = Eigen::Vector4f(0, 0, 960, 1080);
	viewer->core->rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;
	
	rightView = viewer->append_core(Eigen::Vector4f(960, 0, 960, 1080));
	viewer->core_list[rightView].rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;


	this->viewer = viewer;
	viewer->plugins.push_back(&menu);

	menu.callback_draw_viewer_menu = [&,viewer]()
	{
		menu.draw_viewer_menu();
		if (ImGui::CollapsingHeader("Texture Colors", ImGuiTreeNodeFlags_DefaultOpen))
		{
			if (ImGui::ColorEdit4("Color 1", viewer->data_list[0].tex_col1.data(),
				ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel) ||
				ImGui::ColorEdit4("Color 2", viewer->data_list[0].tex_col2.data(),
					ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_PickerHueWheel))
				viewer->data_list[0].grid_texture();
			ImGui::DragFloat("Texture Size", &texture_size, 0.25f);
		}
	};
	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiSetCond_FirstUseEver);
		ImGui::Begin(
			"Solver", nullptr,
			ImGuiWindowFlags_NoSavedSettings
		);
		if (ImGui::Button("Start")) 
			start_solver_thread();
		ImGui::SameLine(); 
		if (ImGui::Button("Stop"))
			stop_solver_thread();


		ImGui::End();
	};

	viewer->core->is_animating = true;
	viewer->core->animation_max_fps = 30.0;
// 	viewer->data.object_scale = 10.0;
}

bool SolverPlugin::load(string filename)
{
	if (solver->is_running)
		stop_solver_thread();

	bool read_obj = false;
	bool read_off = false;

	string file_format = filename.substr(filename.length() - 3, 3);
	if (file_format.compare("obj") == 0)
	{
		if (!igl::readOBJ(filename, V, F))
		{
			cerr << "Failed to load mesh: " << filename << endl;
			return false;
		}
	}
	else if (file_format.compare("off") == 0)
	{
		if (!igl::readOFF(filename, V, F))
		{
			cerr << "Failed to load mesh: " << filename << endl;
			return false;
		}
	}
	else
	{
		cerr << "Unknown file format " << filename << endl;
		return false;
	}

	initialize();
	viewer->coreDataPairs.clear();
	viewer->coreDataPairs.insert({ 0,1 });
	viewer->coreDataPairs.insert({ 1,0 });
	viewer->selected_data_index = 0;
	mesh_loaded = true;	
	mesh_filename = filename;

	return true;
}

void SolverPlugin::initialize()
{
	if (solver->is_running)
		solver->stop();

	while (solver->is_running);

	if (V.rows() == 0 || F.rows() == 0)
		return;

	// initialize the energy
	totalObjective->symDirichlet->V = V.leftCols(2);
	totalObjective->symDirichlet->F = F;
	totalObjective->init();
	// initialize the solver
	solver->init(totalObjective,Map<const VectorXd>(V.data(),V.rows()*2));

	if (processed_mesh_id != 0) {
		viewer->data_list[processed_mesh_id].clear();
		viewer->data_list[source_mesh_id].clear();
	}
	else
	{
		processed_mesh_id = viewer->append_mesh();
	}
	viewer->data_list[source_mesh_id].set_mesh(V, F);
	viewer->data_list[source_mesh_id].set_uv(V);
	viewer->data_list[processed_mesh_id].set_mesh(V, F);

	RGBColors = V;
	RGBColors.rowwise() -= RGBColors.colwise().minCoeff();
	RGBColors *= RGBColors.colwise().maxCoeff().cwiseInverse().asDiagonal();

	viewer->data_list[processed_mesh_id].F;
	viewer->data_list[processed_mesh_id].set_colors(Eigen::MatrixX3d::Ones(F.rows(), 3));

	// init colors of deformed mesh
	uv_triangle_colors = Eigen::MatrixX3d::Ones(F.rows(), 3);
	viewer->data_list[source_mesh_id].set_colors(RVec3(1., 1., 1.));
}

void SolverPlugin::export_uv_to_obj()
{

}

void SolverPlugin::start_solver_thread()
{
	cout << "start new solver" << endl;
	solver_thread = thread(&Solver::run, solver.get());
	solver_thread.detach();
}

void SolverPlugin::stop_solver_thread()
{
	
}

bool SolverPlugin::key_down(int key, int modifiers)
{
	switch (key)
	{
	case GLFW_KEY_T:
		// toggle texturing
		viewer->data_list[source_mesh_id].show_texture = !viewer->data_list[source_mesh_id].show_texture;
		return true; // dont trigger fill variable
	}
	return false;
}

bool SolverPlugin::key_up(int key, int modifiers)
{
	return false;
}

bool SolverPlugin::pre_draw()
{
	if (solver->progressed || update_colors)
		update_mesh();
	return false;
}

void SolverPlugin::update_mesh()
{
	Eigen::VectorXd X;
	solver->get_data(X);
	Map<MatrixX2d> V(X.data(), X.rows()/2, 2);
	viewer->data_list[processed_mesh_id].set_vertices(V);

	// set UV of 3d mesh with newX vertices
	// prepare first for 3d mesh soup
	viewer->data_list[source_mesh_id].set_uv(texture_size * V);


	if (colorByRGB)
		uv_triangle_colors = RGBColors;
	else
		uv_triangle_colors = Eigen::MatrixX3d::Ones(F.rows(), 3);

// 	viewer->get_mesh(uv_id).set_normals(viewer->get_mesh(mesh_id).F_normals);
// 	viewer->get_mesh(uv_id).dirty |= viewer->get_mesh(uv_id).DIRTY_NORMAL;

	Eigen::MatrixX3d uv_sep_colors = Eigen::MatrixX3d::Ones(uv_triangle_colors.rows(), 3);
	Vec vals = Vec::Zero(3 * F.rows());


	Eigen::MatrixX3d uv_dist_colors = Eigen::MatrixX3d::Ones(uv_triangle_colors.rows(), 3);
	if (show_distortion_error)
	{
		Vec dist_vals = totalObjective->symDirichlet->Efi;

		// new dist color impl
		Vec dist_err = dist_vals.transpose().array() - 4.;

		// scale to [0, dist_cutoff]
		dist_err = dist_err / *dist_color_clamp;
		// map > 1 -> 1
		dist_err= 1 - dist_err.unaryExpr([&](double val) { return (val > 1) ? 1 : val; }).array();
		
		uv_dist_colors.col(1) = uv_dist_colors.col(1).cwiseProduct(dist_err);
		uv_dist_colors.col(2) = uv_dist_colors.col(2).cwiseProduct(dist_err);
	}


	uv_triangle_colors.array() *= uv_dist_colors.array();
	

	Vec3i face;
	// draw this first, so a later activation of the hover triangle
	// also overwrites the coloring
	if (hovered_triangle != -1)
	{
		// uv
		face = solver->F.row(hovered_triangle);
		uv_triangle_colors.row(face(0)) << C_hover;
		uv_triangle_colors.row(face(1)) << C_hover;
		uv_triangle_colors.row(face(2)) << C_hover;
		// 3d mesh
		mesh_triangle_colors.row(hovered_triangle) << C_hover;
	}


	viewer->data_list[processed_mesh_id].points.resize(0, Eigen::NoChange);

	viewer->data_list[processed_mesh_id].set_colors(uv_triangle_colors);
	viewer->data_list[source_mesh_id].set_colors(uv_triangle_colors);

	viewer->data_list[processed_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_AMBIENT;
	viewer->data_list[source_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_AMBIENT;

	update_colors = false;
}

bool SolverPlugin::mouse_down(int button, int modifier)
{
	return false;
}

bool SolverPlugin::mouse_up(int button, int modifier)
{
	return true;
}

bool SolverPlugin::mouse_scroll(float delta_y)
{
	return false;
}

bool SolverPlugin::mouse_move(int mouse_x, int mouse_y)
{

	return false;
}

bool SolverPlugin::process_mouse_move()
{
	return false;
}
