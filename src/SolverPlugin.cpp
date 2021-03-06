#include "SolverPlugin.h"

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

using namespace Eigen;

SolverPlugin::SolverPlugin()
	: solver(make_unique<Newton>()), totalObjective(make_shared<TotalObjective>()),
	HandlesInd(totalObjective->constraintsPositional.ConstrainedVerticesInd),
	HandlesPosDeformed(totalObjective->constraintsPositional.ConstrainedVerticesPos)
{

}

void SolverPlugin::init(Viewer *viewer)
{
	leftView = viewer->core().id;
	viewer->core().background_color << 1., 1., 1., 1.; // 0.25, 0.25, 0.25, 1.0;
	viewer->core().orthographic = true;
	viewer->core().viewport = Eigen::Vector4f(0, 0, 960, 1080);
	viewer->core().rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;
	rightView = viewer->append_core(Eigen::Vector4f(960, 0, 960, 1080));
	viewer->core(rightView).rotation_type = igl::opengl::ViewerCore::ROTATION_TYPE_NO_ROTATION;

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

	viewer->core().is_animating = true;
	viewer->core().animation_max_fps = 30.0;
// 	viewer->data.object_scale = 10.0;
}

void SolverPlugin::post_resize(int w, int h)
{
	if (viewer)
	{
		viewer->core(leftView).viewport = Eigen::Vector4f(0, 0, w / 2, h);
		viewer->core(rightView).viewport = Eigen::Vector4f(w / 2 + 1, 0, w / 2, h);
	}
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

	initializeSolver();

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
    viewer->data_list[source_mesh_id].set_colors(Eigen::MatrixX3d::Ones(F.rows(), 3));

    viewer->data_list[processed_mesh_id].set_mesh(V, F);
    viewer->data_list[processed_mesh_id].set_colors(Eigen::MatrixX3d::Ones(F.rows(), 3));
    viewer->data(source_mesh_id).set_visible(false, leftView);
    viewer->data(source_mesh_id).point_size = 10;

    viewer->data(processed_mesh_id).set_visible(false, rightView);
    viewer->data(processed_mesh_id).point_size = 10;

	mesh_loaded = true;	
	mesh_filename = filename;

	return true;
}

void SolverPlugin::initializeSolver()
{
	if (solver->is_running)
		solver->stop();

	while (solver->is_running);

	if (V.rows() == 0 || F.rows() == 0)
		return;

	// initialize the energy
	totalObjective->symDirichlet.V = V.leftCols(2);
	totalObjective->symDirichlet.F = F;
	totalObjective->constraintsPositional.numV = V.rows();

	totalObjective->init();
	// initialize the solver
	VectorXd XX = Map<const VectorXd>(V.data(), V.rows() * 2);
	solver->init(totalObjective, XX);
	solver->setFlipAvoidingLineSearch(F);
    start_solver_thread();
}

bool SolverPlugin::pre_draw()
{
	if (solver->progressed || update_colors)
		update_mesh();
	return false;
}

void SolverPlugin::update_mesh()
{
	VectorXd X;
	solver->get_data(X);
	MatrixX3d V(X.rows() / 2, 3);
	V.leftCols(2) = Map<MatrixX2d>(X.data(), X.rows() / 2, 2);
	V.rightCols(1).setZero();
	
    viewer->data_list[processed_mesh_id].set_vertices(V);

	// set UV of 3d mesh with newX vertices
	// prepare first for 3d mesh soup
	viewer->data_list[source_mesh_id].set_uv(texture_size * V.leftCols(2));


	if (colorByRGB)
		uv_triangle_colors = RGBColors;
	else
		uv_triangle_colors = Eigen::MatrixX3d::Ones(F.rows(), 3);

	// 	viewer->get_mesh(uv_id).set_normals(viewer->get_mesh(mesh_id).F_normals);
	// 	viewer->get_mesh(uv_id).dirty |= viewer->get_mesh(uv_id).DIRTY_NORMAL;

	Eigen::MatrixX3d uv_sep_colors = Eigen::MatrixX3d::Ones(uv_triangle_colors.rows(), 3);
    VectorXd vals = VectorXd::Zero(3 * F.rows());


	Eigen::MatrixX3d uv_dist_colors = Eigen::MatrixX3d::Ones(uv_triangle_colors.rows(), 3);
	if (show_distortion_error)
	{
        VectorXd dist_vals = totalObjective->symDirichlet.Efi;

		// new dist color impl
        VectorXd dist_err = dist_vals.transpose().array() - 4.;

		// scale to [0, dist_cutoff]
		dist_err = dist_err / *dist_color_clamp;
		// map > 1 -> 1
		dist_err = 1 - dist_err.unaryExpr([&](double val) { return (val > 1) ? 1 : val; }).array();

		uv_dist_colors.col(1) = uv_dist_colors.col(1).cwiseProduct(dist_err);
		uv_dist_colors.col(2) = uv_dist_colors.col(2).cwiseProduct(dist_err);
	}


	uv_triangle_colors.array() *= uv_dist_colors.array();



    viewer->data_list[source_mesh_id].points.resize(HandlesInd.size(), 6);
    viewer->data_list[source_mesh_id].points.setZero();
    viewer->data_list[source_mesh_id].points.leftCols(2) = HandlesPosSource;
    viewer->data_list[source_mesh_id].points.col(2).setConstant(1);
    viewer->data_list[source_mesh_id].points.col(5).setConstant(1);
    viewer->data_list[source_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_POINTS;


	viewer->data_list[processed_mesh_id].points.resize(HandlesInd.size(), 6);
	viewer->data_list[processed_mesh_id].points.setZero();
	viewer->data_list[processed_mesh_id].points.leftCols(2) = HandlesPosDeformed;
	viewer->data_list[processed_mesh_id].points.col(2).setConstant(1);
	viewer->data_list[processed_mesh_id].points.col(5).setConstant(1);
	viewer->data_list[processed_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_OVERLAY_POINTS;

	viewer->data_list[processed_mesh_id].set_colors(uv_triangle_colors);
	viewer->data_list[source_mesh_id].set_colors(uv_triangle_colors);

	viewer->data_list[processed_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_AMBIENT;
	viewer->data_list[source_mesh_id].dirty |= igl::opengl::MeshGL::DIRTY_AMBIENT;

	update_colors = false;
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
	cout << "stopping solver" << endl;
	solver->stop();
	cout << "solver stopped" << endl;
}

int SolverPlugin::FindHitVertex()
{
    MatrixX3d V; MatrixX3i F;
    bool leftViewChosen = viewer->selected_core_index == viewer->core_index(leftView);
    if (leftViewChosen)
    {
        V = viewer->data_list[processed_mesh_id].V;
        F = viewer->data_list[processed_mesh_id].F;
    }
    else
    {
        V = viewer->data_list[source_mesh_id].V;
        F = viewer->data_list[source_mesh_id].F;
    }
	int fid;
	Eigen::Vector3f BC;
	// Cast a ray in the view direction starting from the mouse position
	double x = viewer->current_mouse_x;
	double y = viewer->core().viewport(3) - viewer->current_mouse_y;
	if(!igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer->core().view * viewer->core().model,
		viewer->core().proj, viewer->core().viewport, V, F, fid, BC))
        return -1;
    if (leftViewChosen)
        viewer->selected_data_index = viewer->mesh_index(processed_mesh_id);
    else
        viewer->selected_data_index = viewer->mesh_index(source_mesh_id);
	int maxBCind;
	double maxBc = BC.maxCoeff(&maxBCind);
	return F(fid, maxBCind);
}

int SolverPlugin::FindHitHandle()
{
	int handleIndex=-1;
	auto &ind = totalObjective->constraintsPositional.ConstrainedVerticesInd;
	Vector3f m = Vector3f(viewer->current_mouse_x, viewer->current_mouse_y, 0);
	m[1] = viewer->core().viewport[3] - m[1];
	// find handle
	float min = std::numeric_limits<float>::max();
	for (int i = 0; i < ind.size(); i++)
	{
		Vector3f hi;
		// << HandlesPos.row(i).cast<float>(), 0.0;
		hi(0) = HandlesPosDeformed(i, 0); hi(1) = HandlesPosDeformed(i, 1); hi(2) = 0;
		Eigen::Vector3f p = igl::project(hi, (viewer->core().view * viewer->core().model).eval(), viewer->core().proj, viewer->core().viewport);
		float dist = (p - m).norm();

		if (dist < viewer->data().point_size / 2.0 && dist < min)
		{
			handleIndex = i;
			min = dist;
		}
	}
	return handleIndex;
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

bool SolverPlugin::mouse_down(int button, int modifier)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
        int vid = FindHitVertex();
		if (modifier == GLFW_MOD_CONTROL)
		{
			if (vid >= 0)
				AddHandle(vid);
		}
		else if (modifier == (GLFW_MOD_ALT))
		{
			int index = FindHitHandle();
			if (index>=0)
				RemoveHandle(index);
		}
		else
            if (viewer->selected_core_index == viewer->core_index(leftView))
			    selectedHandle = FindHitHandle();

		return true; // disable trackball rotation
	}
	return false;
}

void SolverPlugin::AddHandle(int &vid)
{
    int mesh_id;
    if (viewer->selected_core_index == viewer->core_index(leftView))
        mesh_id = processed_mesh_id;
    else
        mesh_id = source_mesh_id;

	solver->wait_for_parameter_update_slot();
	HandlesInd.push_back(vid);
	HandlesPosDeformed.conservativeResize(HandlesPosDeformed.rows() + 1, NoChange);
	HandlesPosDeformed.bottomRows(1) = viewer->data_list[processed_mesh_id].V.row(vid).leftCols(2);

    HandlesPosSource.conservativeResize(HandlesPosSource.rows() + 1, NoChange);
    HandlesPosSource.bottomRows(1) = viewer->data_list[source_mesh_id].V.row(vid).leftCols(2);

	solver->release_parameter_update_slot();
}

void SolverPlugin::RemoveHandle(int index)
{
	solver->wait_for_parameter_update_slot();
	HandlesInd.erase(HandlesInd.begin() + index);
	HandlesPosDeformed.block(index, 0, HandlesPosDeformed.rows() - index - 1, 2) = HandlesPosDeformed.block(index + 1, 0, HandlesPosDeformed.rows() - index - 1, 2);
	HandlesPosDeformed.conservativeResize(HandlesPosDeformed.rows() - 1, 2);

    HandlesPosSource.block(index, 0, HandlesPosSource.rows() - index - 1, 2) = HandlesPosSource.block(index + 1, 0, HandlesPosSource.rows() - index - 1, 2);
    HandlesPosSource.conservativeResize(HandlesPosSource.rows() - 1, 2);

	solver->release_parameter_update_slot();
}

bool SolverPlugin::mouse_up(int button, int modifier)
{
	selectedHandle = -1;
	return true;
}

void SolverPlugin::shutdown()
{
    stop_solver_thread();
}

bool SolverPlugin::mouse_scroll(float delta_y)
{
	return false;
}

bool SolverPlugin::mouse_move(int mouse_x, int mouse_y)
{
	if (selectedHandle >= 0)
		MoveHandle(mouse_x, mouse_y);

	return false;
}

void SolverPlugin::MoveHandle(int mouse_x, int mouse_y)
{
	Vector3d mouse_pos, projected_pos;
	mouse_pos << mouse_x, viewer->core().viewport[3] - mouse_y, 0.;
	igl::unproject(mouse_pos, viewer->core().view * viewer->core().model, viewer->core().proj, viewer->core().viewport, projected_pos);
	
// 	solver->wait_for_parameter_update_slot();
	HandlesPosDeformed.row(selectedHandle) = projected_pos.head(2);
// 	solver->release_parameter_update_slot();
}

bool SolverPlugin::process_mouse_move()
{
	return false;
}
