#pragma once
#ifndef GIZMOMENU_H
#define GIZMOMENU_H

#include <ImGuizmo/ImGuizmo.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/material_colors.h>
#include <imgui/imgui.h>
#include <iostream>

class GizmoMenu : public igl::opengl::glfw::imgui::ImGuiMenu
{
public:
	virtual bool pre_draw() override {
		ImGuiMenu::pre_draw();
		ImGuizmo::BeginFrame();
		return false;
	}

	virtual bool post_draw() override {
		ImGuiMenu::post_draw();
		return false;
	}

	virtual void draw_custom_window() override {
		static Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
		matrix(3, 0) = -0.4;
		matrix(3, 1) = 0.4;
		float objectMatrix[16] =
		{ 1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			0.4f, 0.f, 0.f, 1.f };

		float cameraView[16] =
		{ 1.f, 0.f, 0.f, 0.f,
			0.f, 1.f, 0.f, 0.f,
			0.f, 0.f, 1.f, 0.f,
			1.f, -1.f, -4.f, 1.f };
		//Eigen::Affine3f rescale = Eigen::Scaling(0.5f * viewer->core.camera_base_zoom)
		//	* Eigen::Translation3f(viewer->core.camera_base_translation);
		Eigen::Affine3f view = Eigen::Affine3f(viewer->core().view) /** rescale.inverse()*/;
		Eigen::Matrix4f proj = viewer->core().proj;

		GizmoMenu::EditTransform(cameraView, proj.data(), objectMatrix);

		Eigen::Affine3f model(matrix);
		/*model = rescale.inverse() * model;*/
	}
private:
	void EditTransform(const float *cameraView, float *cameraProjection, float* matrix)
	{
		static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::ROTATE);
		static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::WORLD);
		static bool useSnap = false;
		static float snap[3] = { 1.f, 1.f, 1.f };

		if (ImGui::IsKeyPressed(90))
			mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
		if (ImGui::IsKeyPressed(69))
			mCurrentGizmoOperation = ImGuizmo::ROTATE;
		if (ImGui::IsKeyPressed(82)) // r Key
			mCurrentGizmoOperation = ImGuizmo::SCALE;
		if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
			mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
		ImGui::SameLine();
		if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
			mCurrentGizmoOperation = ImGuizmo::ROTATE;
		ImGui::SameLine();
		if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
			mCurrentGizmoOperation = ImGuizmo::SCALE;
		float matrixTranslation[3], matrixRotation[3], matrixScale[3];
		ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);
		ImGui::InputFloat3("Tr", matrixTranslation, 3);
		ImGui::InputFloat3("Rt", matrixRotation, 3);
		ImGui::InputFloat3("Sc", matrixScale, 3);
		ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

		if (mCurrentGizmoOperation != ImGuizmo::SCALE)
		{
			if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
				mCurrentGizmoMode = ImGuizmo::LOCAL;
			ImGui::SameLine();
			if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
				mCurrentGizmoMode = ImGuizmo::WORLD;
		}
		if (ImGui::IsKeyPressed(83))
			useSnap = !useSnap;
		ImGui::Checkbox("", &useSnap);
		ImGui::SameLine();

		switch (mCurrentGizmoOperation)
		{
		case ImGuizmo::TRANSLATE:
			ImGui::InputFloat3("Snap", &snap[0]);
			break;
		case ImGuizmo::ROTATE:
			ImGui::InputFloat("Angle Snap", &snap[0]);
			break;
		case ImGuizmo::SCALE:
			ImGui::InputFloat("Scale Snap", &snap[0]);
			break;
		}
		ImGuiIO& io = ImGui::GetIO();
		ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
		ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, matrix, NULL, useSnap ? &snap[0] : NULL);
	}
};

#endif