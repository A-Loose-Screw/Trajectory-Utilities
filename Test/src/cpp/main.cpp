#include <iostream>
#include <TrajectoryUtilities.h>

// UI
#include <UI.h>

Trajectory<Splines::CatmullRom> trajectory;


UI ui;

bool toolActive = true;
float colour[4];

void onUpdate() {
  ImGui::Begin("Trajectory Tool", &toolActive, ImGuiWindowFlags_MenuBar);
  // if (ImGui::BeginMenuBar()) {
  //   if (ImGui::BeginMenu("File")) {
  //     if (ImGui::MenuItem("Open...", "Ctrl+O")) { /* Do stuff */ }
  //     if (ImGui::MenuItem("Save", "Ctrl+S"))   { /* Do stuff */ }
  //     if (ImGui::MenuItem("Close", "Ctrl+W"))  {
  //       std::cout << "Closing tool" << std::endl;
  //       toolActive = false;
  //     }
  //     ImGui::EndMenu();
  //   }
  //   ImGui::EndMenuBar();
  // }

  // const float my_values[] = { 0.2f, 0.1f, 1.0f, 0.5f, 0.9f, 2.2f };
  
  // ImGui::PlotLines("X values", my_values, IM_ARRAYSIZE(my_values));

  ImDrawList *list = ImGui::GetWindowDrawList();
  ImVec2 p = ImGui::GetCursorScreenPos();
  const ImU32 col32 = ImColor(ImVec4(1.0f, 1.0f, 0.4f, 1.0f));

  for (double step = 0; step < trajectory.getRawTrajectory().totalLength; step += 0.001) {
    Splines::Waypoint point = trajectory.getCoords(step);
    list->AddCircle(ImVec2(p.x + (point.x*100), p.y + (point.y*100)), 1, col32, 20, 1);
  }

  // list->AddCircle(ImVec2(p.x+100,p.y+100), 5, col32, 20, 1);
  // ImGui::ColorEdit4("Colour", colour);

  ImGui::End();
}

int main() {
  trajectory.push_back({
    {0,4}, // Start Ctrl Pnt

    {1,4}, {3,5}, // forward
    {3,6}, {1,4}, // back
    
    {1,3} // End Ctrl Pnt
  });
  

  trajectory.build();
  trajectory.simulate();

  // trajectory.print();

  while (!ui.windowCloseEvent()) {
    ui.preUpdate();
    onUpdate();
    ui.postUpdate();
  }

  return 0;
}