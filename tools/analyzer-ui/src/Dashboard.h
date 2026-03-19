#pragma once
#include "CSVParser.h"
#include <string>
#include <vector>

class Dashboard {
public:
  Dashboard();
  ~Dashboard();

  void loadCSV(const std::string &filename);
  void render();

private:
  CSVData m_data;
  bool m_hasData = false;

  float m_currentTimeUs = 0.0f;
  float m_maxTimeUs = 0.0f;
  bool m_playing = false;
  float m_playbackSpeed = 1.0f;

  float m_lastFrameTime = 0.0f;

  std::string m_colX;
  std::string m_colY;
  std::string m_colTheta;

  void updatePlayback(float deltaTime);

  void renderControlPanel();
  void renderGridWindow();
  void renderGraphsWindow();
  void renderStateWindow();

  int getIndexForTime(float timeUs) const;
};
