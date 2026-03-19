#include "Dashboard.h"
#include "imgui.h"
#include "implot.h"
#include <algorithm>
#include <math.h>

Dashboard::Dashboard() {}
Dashboard::~Dashboard() {}

void Dashboard::loadCSV(const std::string &filename) {
  if (CSVParser::parse(filename, m_data)) {
    m_hasData = !m_data.timestamps.empty();
    if (m_hasData) {
      m_maxTimeUs = m_data.timestamps.back();
      m_currentTimeUs = m_data.timestamps.front();
      m_playing = false;

      m_colX = "";
      m_colY = "";
      m_colTheta = "";
      for (const auto &h : m_data.headers) {
        if (h.find("/x") != std::string::npos && m_colX.empty())
          m_colX = h;
        if (h.find("/y") != std::string::npos && m_colY.empty())
          m_colY = h;
        if (h.find("/theta") != std::string::npos && m_colTheta.empty())
          m_colTheta = h;
      }
    }
  }
}

int Dashboard::getIndexForTime(float timeUs) const {
  if (!m_hasData || m_data.timestamps.empty())
    return 0;
  auto it = std::upper_bound(m_data.timestamps.begin(), m_data.timestamps.end(),
                             timeUs);
  if (it == m_data.timestamps.begin())
    return 0;
  return std::distance(m_data.timestamps.begin(), it) - 1;
}

void Dashboard::updatePlayback(float deltaTime) {
  if (!m_playing || !m_hasData)
    return;
  m_currentTimeUs += deltaTime * 1000000.0f * m_playbackSpeed;
  if (m_currentTimeUs >= m_maxTimeUs) {
    m_currentTimeUs = m_maxTimeUs;
    m_playing = false;
  }
}

void Dashboard::render() {
  float currentFrameTime = ImGui::GetTime();
  float deltaTime = currentFrameTime - m_lastFrameTime;
  m_lastFrameTime = currentFrameTime;

  updatePlayback(deltaTime);

  renderControlPanel();
  if (m_hasData) {
    renderGridWindow();
    renderGraphsWindow();
    renderStateWindow();
  }
}

void Dashboard::renderControlPanel() {
  ImGui::Begin("Controls");

  static char csvPath[256] = "log_001.csv";
  ImGui::InputText("CSV Path", csvPath, 256);
  if (ImGui::Button("Load CSV")) {
    loadCSV(csvPath);
  }

  if (!m_hasData) {
    ImGui::Text("No data loaded.");
    ImGui::End();
    return;
  }

  ImGui::Separator();
  if (ImGui::Button(m_playing ? "Pause" : "Play")) {
    m_playing = !m_playing;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset")) {
    m_currentTimeUs = m_data.timestamps.empty() ? 0 : m_data.timestamps.front();
    m_playing = false;
  }

  ImGui::SliderFloat("Speed", &m_playbackSpeed, 0.1f, 10.0f, "%.1fx");
  ImGui::SliderFloat("Timeline (us)", &m_currentTimeUs,
                     m_data.timestamps.front(), m_maxTimeUs, "%.0f");

  ImGui::Separator();
  ImGui::Text("2D Grid Mappings:");

  auto combobox = [&](const char *label, std::string &current) {
    if (ImGui::BeginCombo(label, current.empty() ? "None" : current.c_str())) {
      if (ImGui::Selectable("None", current.empty()))
        current = "";
      for (const auto &h : m_data.headers) {
        if (h == "timestamp_us")
          continue;
        if (ImGui::Selectable(h.c_str(), current == h))
          current = h;
      }
      ImGui::EndCombo();
    }
  };

  combobox("X Coordinate", m_colX);
  combobox("Y Coordinate", m_colY);
  combobox("Theta (Heading)", m_colTheta);

  ImGui::End();
}

void Dashboard::renderGridWindow() {
  ImGui::Begin("2D Grid");

  if (m_colX.empty() || m_colY.empty()) {
    ImGui::Text("Please select X and Y coordinates in the Control Panel.");
    ImGui::End();
    return;
  }

  int idx = getIndexForTime(m_currentTimeUs);
  double curX = m_data.columns[m_colX][idx];
  double curY = m_data.columns[m_colY][idx];
  double curTheta = m_colTheta.empty() ? 0.0 : m_data.columns[m_colTheta][idx];

  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImVec2 size = ImGui::GetContentRegionAvail();

  float canvas_w = size.x;
  float canvas_h = size.y;
  if (canvas_w < 50.0f)
    canvas_w = 50.0f;
  if (canvas_h < 50.0f)
    canvas_h = 50.0f;

  draw_list->AddRectFilled(p, ImVec2(p.x + canvas_w, p.y + canvas_h),
                           IM_COL32(30, 30, 30, 255));
  draw_list->AddRect(p, ImVec2(p.x + canvas_w, p.y + canvas_h),
                     IM_COL32(255, 255, 255, 255));

  float cell_w = canvas_w / 16.0f;
  float cell_h = canvas_h / 16.0f;

  for (int i = 0; i < 17; i++) {
    draw_list->AddLine(ImVec2(p.x + i * cell_w, p.y),
                       ImVec2(p.x + i * cell_w, p.y + canvas_h),
                       IM_COL32(100, 100, 100, 100));
    draw_list->AddLine(ImVec2(p.x, p.y + i * cell_h),
                       ImVec2(p.x + canvas_w, p.y + i * cell_h),
                       IM_COL32(100, 100, 100, 100));
  }

  float scaleX = canvas_w / 2.88f;
  float scaleY = canvas_h / 2.88f;

  float screenX = p.x + (curX * scaleX);
  float screenY = p.y + canvas_h - (curY * scaleY);

  float radius = (cell_w > cell_h ? cell_h : cell_w) * 0.4f;
  draw_list->AddCircleFilled(ImVec2(screenX, screenY), radius,
                             IM_COL32(0, 255, 0, 255));

  float ptX = screenX + cos(curTheta) * radius * 1.5f;
  float ptY = screenY - sin(curTheta) * radius * 1.5f;
  draw_list->AddLine(ImVec2(screenX, screenY), ImVec2(ptX, ptY),
                     IM_COL32(255, 0, 0, 255), 2.0f);

  if (m_data.columns.count(m_colX) && m_data.columns.count(m_colY)) {
    ImVec2 lastPt;
    bool hasLast = false;
    for (int i = 0; i <= idx; i++) {
      float trajX = p.x + (m_data.columns[m_colX][i] * scaleX);
      float trajY = p.y + canvas_h - (m_data.columns[m_colY][i] * scaleY);
      if (hasLast) {
        draw_list->AddLine(lastPt, ImVec2(trajX, trajY),
                           IM_COL32(0, 150, 255, 100));
      }
      lastPt = ImVec2(trajX, trajY);
      hasLast = true;
    }
  }

  ImGui::End();
}

void Dashboard::renderGraphsWindow() {
  ImGui::Begin("Sensor Graphs (ImPlot)");

  if (ImPlot::BeginPlot("Telemetry Viewer", ImVec2(-1, -1))) {
    ImPlot::SetupAxes("Time (us)", "Value", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);

    for (const auto &kv : m_data.columns) {
      if (kv.first.find("IR") != std::string::npos ||
          kv.first.find("front") != std::string::npos ||
          kv.first.find("side") != std::string::npos) {
        if (!kv.second.empty() && !m_data.timestamps.empty()) {
          ImPlot::PlotLine(kv.first.c_str(), m_data.timestamps.data(),
                           kv.second.data(), m_data.timestamps.size());
        }
      }
    }

    double dragTime = m_currentTimeUs;
    ImPlot::DragLineX(0, &dragTime, ImVec4(1, 1, 1, 0.5f), 1.0f);
    m_currentTimeUs = dragTime;

    if (m_currentTimeUs < m_data.timestamps.front())
      m_currentTimeUs = m_data.timestamps.front();
    if (m_currentTimeUs > m_maxTimeUs)
      m_currentTimeUs = m_maxTimeUs;

    ImPlot::EndPlot();
  }
  ImGui::End();
}

void Dashboard::renderStateWindow() {
  ImGui::Begin("Robot State");

  int idx = getIndexForTime(m_currentTimeUs);

  if (ImGui::BeginTable("StateTable", 2,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Variable");
    ImGui::TableSetupColumn("Value");
    ImGui::TableHeadersRow();

    for (const auto &kv : m_data.columns) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("%s", kv.first.c_str());
      ImGui::TableSetColumnIndex(1);
      ImGui::Text("%.4f", kv.second[idx]);
    }
    ImGui::EndTable();
  }

  ImGui::End();
}
